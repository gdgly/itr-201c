module key_matrix_4x4
(
	clk_1mhz, reset_n, key_in, key_out, 
	irq_key, key_code_out
);

input clk_1mhz;			//input clock = 1MHz
input reset_n;
input  [3:0] key_in;
output [3:0] key_out;
output irq_key;
output [7:0] key_code_out;

//---------------------------------------------------------------------------------
`define KEY_CLK_WIDTH	13
wire [`KEY_CLK_WIDTH - 1:0] key_cnt;
wire key_clk;

my_counter C1(.aclr(!reset_n), .clock(clk_1mhz), .q(key_cnt));		//key_clk = 8ms
assign key_clk = key_cnt[`KEY_CLK_WIDTH - 1];

//---------------------------------------------------------------------------------
reg [3:0] key_shift;
wire key_shift_en;
wire [7:0] key_code;

assign key_shift_en = &key_in;

always @(posedge key_clk or negedge reset_n)
begin
	if(!reset_n)          key_shift <= 4'b1110;
	else if(key_shift_en) key_shift <= {key_shift[0], key_shift[3:1]};
end

assign key_out  = key_shift;
assign key_code = key_shift_en ? 8'hff : {key_in, key_shift};

//---------------------------------------------------------------------------------
reg [7:0] key_curr_state;
reg [7:0] key_next_state;

always @(posedge key_clk or negedge reset_n)
begin
	if(!reset_n) key_curr_state <= 8'b0;
	else         key_curr_state <= key_code;
end


always @(posedge key_clk or negedge reset_n)
begin
	if(!reset_n) 
		key_next_state <= 8'b0;
	else if(key_curr_state != key_next_state) 		
		key_next_state <= key_curr_state;
end

assign irq_key      = !((key_in != 4'b1111) && (key_curr_state != key_next_state));
assign key_code_out = key_curr_state;


endmodule
