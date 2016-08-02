module wsd
(
	wsd_start, wsd_clk, reset_n, wsd_in, q, data_ready, state
);

input wsd_start;
input wsd_clk;				//current clock is 1MHz(1us)
input reset_n;
input wsd_in;
output [39:0] q;			//16 bit humidity, 16 bit temperature, and 8 bit verify.
output data_ready;
output [2:0] state;


//----------------------------------------------------------------------------------------------
//	Catch wsd_start HIGH edge
//----------------------------------------------------------------------------------------------
reg wsd_sample0, wsd_sample1;
wire wsd_sample_high_edge;

always @(posedge wsd_clk or negedge reset_n)
begin
	if(!reset_n) begin
		wsd_sample0 <= 1'b0;
		wsd_sample1 <= 1'b0;
	end
	else begin
		wsd_sample0 <= wsd_start;
		wsd_sample1 <= wsd_sample0;
	end
end

assign wsd_sample_high_edge = wsd_sample0 && !wsd_sample1;

//----------------------------------------------------------------------------------------------
//	When catch wsd_sample_high_edge, delay 50us, and then start sampling.
//----------------------------------------------------------------------------------------------
`define DELAY_SUM		50
reg [5:0] delay_counter;
reg delay;
reg start_sample;

always @(posedge wsd_clk or negedge reset_n)
begin
	if(!reset_n) begin
		delay_counter <= 6'b0;
		delay         <= 1'b0;
		start_sample  <= 1'b0;
	end
	else begin
		if(wsd_sample_high_edge) begin
			delay <= 1'b1;
		end
		else if(delay) begin
			if(delay_counter >= `DELAY_SUM) begin
				delay <= 1'b0;
				delay_counter <= 6'b0;
				start_sample <= 1'b1;
			end
			else begin
				delay_counter <= delay_counter + 6'b1;
			end
		end
		else if(start_sample)
			start_sample <= 1'b0;
	end
end


//----------------------------------------------------------------------------------------------
//	Catch wsd_in's edge
//----------------------------------------------------------------------------------------------
reg [1:0] wsd;
wire wsd_posedge;
wire wsd_negedge;

always @(posedge wsd_clk or negedge reset_n)
begin
	if(!reset_n) begin
		wsd[0] <= 1'b0;
		wsd[1] <= 1'b0;
	end
	else begin
		wsd[0] <= wsd_in;
		wsd[1] <= wsd[0];
	end
end

assign wsd_posedge =  wsd[0] && !wsd[1];
assign wsd_negedge = !wsd[0] &&  wsd[1];

//----------------------------------------------------------------------------------------------
//	WSD state machine
//----------------------------------------------------------------------------------------------
`define DATA_SUM		41
`define TIME_OUT		250
reg [2:0] curr_state;
reg [2:0] next_state;
parameter [2:0] WSD_IDLE = 3'b0, WSD_ST1 = 3'b1, WSD_ST2 = 3'b010, 
                WSD_END = 3'b011, WSD_DATA = 3'b100, WSD_ERR = 3'b101;

always @(posedge wsd_clk or negedge reset_n)
begin
	if(!reset_n) curr_state <= WSD_IDLE;
	else         curr_state <= next_state;			
end

always @(*)
begin
	case(curr_state)
	
		WSD_IDLE:begin
			if(start_sample)  next_state = WSD_ST1;
			else           	  next_state = WSD_IDLE;
		end
		
		WSD_ST1:begin
			if(wsd_posedge)                          next_state = WSD_ST2;
			else if(wsd_start_time_out > `TIME_OUT)  next_state = WSD_ERR;
			else                                     next_state = WSD_ST1;
		end
		
		WSD_ST2:begin
			if(wsd_negedge)                    next_state = WSD_END;
			else if(wsd_counter > `TIME_OUT)   next_state = WSD_ERR;			
			else                               next_state = WSD_ST2;
		end
		
		WSD_END:begin
			if(data_counter >= `DATA_SUM)  next_state = WSD_DATA;
			else                           next_state = WSD_ST1;
		end
		
		WSD_DATA:			              next_state = WSD_IDLE;		
		WSD_ERR:		                      next_state = WSD_IDLE; 
		default:			              next_state = WSD_IDLE;		
	endcase
end

//----------------------------------------------------------------------------------------------
//	counter
//----------------------------------------------------------------------------------------------
reg [7:0] wsd_counter;

always @(posedge wsd_clk or negedge reset_n)
begin
	if(!reset_n) 
		wsd_counter <= 8'b0;
	else begin 
		if(curr_state == WSD_ST2) 
			wsd_counter <= wsd_counter + 8'b1;
		else if(curr_state == WSD_END)
			wsd_counter <= wsd_counter;
		else
			wsd_counter <= 8'b0;
	end
end

//----------------------------------------------------------------------------------------------
//	counter if time out
//----------------------------------------------------------------------------------------------
reg [7:0] wsd_start_time_out;

always @(posedge wsd_clk or negedge reset_n)
begin
	if(!reset_n) 
		wsd_start_time_out <= 8'b0;
	else if(curr_state == WSD_ST1) 
		wsd_start_time_out <= wsd_start_time_out + 8'b1;		
	else
		wsd_start_time_out <= 8'b0;	
end

//----------------------------------------------------------------------------------------------
reg [`DATA_SUM - 1:0] data_shift;
reg [5:0] data_counter;

always @(posedge wsd_clk or negedge reset_n)
begin
	if(!reset_n) begin
		data_shift   <= `DATA_SUM'b0;
		data_counter <= 6'b0;		
	end
	else if(curr_state == WSD_DATA) begin
		data_counter <= 6'b0;		
	end
	else if(curr_state == WSD_END) begin
		if((wsd_counter >= 75) && (wsd_counter < 90)) begin	//data start: 80us			
			data_shift <= {data_shift[39:0], 1'b1};
			data_counter <= data_counter + 6'b1;
		end
		else if((wsd_counter >= 65) && (wsd_counter < 75)) begin	//data 1: 70us
			data_shift <= {data_shift[39:0], 1'b1};
			data_counter <= data_counter + 6'b1;
		end
		else if((wsd_counter >= 15) && (wsd_counter <= 35)) begin	//data 0: 26~28us
			data_shift <= {data_shift[39:0], 1'b0};
			data_counter <= data_counter + 6'b1;
		end
	end	
end

//----------------------------------------------------------------------------------------------
reg [39:0] data_out_reg;
reg data_ready_reg;

always @(posedge wsd_clk or negedge reset_n)
begin
	if(!reset_n)
		data_out_reg <= 40'b0;			
	else if(curr_state == WSD_DATA)
		data_out_reg <= data_shift[40:1];
	else if(curr_state == WSD_ERR)
		data_out_reg <= {40{1'b1}};
end

//always @(posedge wsd_clk or negedge reset_n)
//begin
//	if(!reset_n)
//		data_ready_reg <= 1'b0;			
//	else if(curr_state == WSD_DATA)
//		data_ready_reg <= 1'b1;
//	else if(curr_state == WSD_ERR)
//		data_ready_reg <= 1'b1;
//	else
//		data_ready_reg <= 1'b0;
//end

always @(posedge wsd_clk or negedge reset_n)
begin
	if(!reset_n)
		data_ready_reg <= 1'b0;			
	else if(curr_state == WSD_DATA)
		data_ready_reg <= 1'b1;
	else if(curr_state == WSD_ERR)
		data_ready_reg <= 1'b1;
	else if(curr_state == WSD_ST1)
		data_ready_reg <= 1'b0;
end

assign q          = data_out_reg;
assign data_ready = data_ready_reg;
assign state      = curr_state;

endmodule
