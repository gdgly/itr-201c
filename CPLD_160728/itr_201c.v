module itr_201c
(

//------------------------------------------------input  signals---------------------------------------------------------
gpmc_cs_n, gpmc_oe_n, gpmc_we_n, sa, fpga_clk, sys_rst_n, led_link, led_speed, cpu_txd, cpu_rxd, wsd, sw_rst_in, clk_1mhz,
cpld_key_in, 

//------------------------------------------------output signals---------------------------------------------------------
io_rdy, irq_cpld, cpld_led, relay_ctrl, wsd_sample, sw_rst_out, wdg_out, buzzer_out, rst_out_p, rst_out_n, 
io_led_out, cpld_key_out, lcd_vled, lvds_pwen, cpld_lcd_nrst, 

//------------------------------------------------inout  signals---------------------------------------------------------
sd

);

`define SA_ADDR_WIDTH	8

input [3:1] gpmc_cs_n;
input gpmc_oe_n;
input gpmc_we_n;
input [`SA_ADDR_WIDTH - 1:0] sa;
input fpga_clk;
input sys_rst_n;
input led_link;
input led_speed;
input [2:1] cpu_txd;
input [2:1] cpu_rxd;
input [2:1] wsd;
input sw_rst_in;
input clk_1mhz;
input [3:0] cpld_key_in;

output io_rdy;
output [3:0] irq_cpld;
output [1:0] cpld_led;
output [3:0] relay_ctrl;
output [2:1] wsd_sample;
output sw_rst_out;
output wdg_out;
output buzzer_out;
output rst_out_p;
output rst_out_n;
output [5:0] io_led_out;
output [3:0] cpld_key_out;
output lcd_vled;
output lvds_pwen;
output cpld_lcd_nrst;


inout [7:0] sd;

//------------------------------------------------------------------------------------------------
// Setup NGCS chip select
// MISC_BANK: 8 bit; KEY_BANK: 8 bit; LAN0_BANK: 16 bit; LAN1_BANK: 16 bit; UART_BANK: 8 bit.
//------------------------------------------------------------------------------------------------
parameter ETH_BANK = 1, UART_BANK = 2, MISC_BANK = 3;
wire [3:1] ngcs;
wire noe;
wire nwe;

assign ngcs = gpmc_cs_n;
assign nwe  = gpmc_we_n;
assign noe  = gpmc_oe_n;
assign io_rdy   = 1'b1;

//------------------------------------------------------------------------------------------------
//Reset out logic
//------------------------------------------------------------------------------------------------
assign rst_out_p     = !sys_rst_n;
assign rst_out_n     =  sys_rst_n;
assign sw_rst_out    = !sw_rst_in ? 1'b0 : 1'bz;
assign cpld_lcd_nrst = sys_rst_n;

//------------------------------------------------------------------------------------------------
// Watchdog logic: 0x2800_0000
// wdg_reg: [0] - Open/close watchdog. 0 - close. 1 - open.
//          [1] - Feed watchdog. 
//------------------------------------------------------------------------------------------------
wire wdg_cs;
reg [23:0] wdg_cnt;
reg wdg_en;
reg wdg_pwm;

assign wdg_cs = !ngcs[MISC_BANK] & (sa[7:0] == 8'b0);

//always @(posedge fpga_clk or negedge sys_rst_n)
always @(posedge fpga_clk, negedge sys_rst_n)
begin
	if(!sys_rst_n) begin
		wdg_en  <= 1'b0;
		wdg_pwm <= 1'b0;
	end
	else if(!nwe && wdg_cs) begin
		{wdg_pwm, wdg_en} <= sd[1:0];
	end
end

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)   wdg_cnt <= 24'b0;
	else            wdg_cnt <= wdg_cnt + 24'b1;
end

assign wdg_out = wdg_en ? wdg_pwm : wdg_cnt[23];
//assign wdg_out = wdg_en ? 1'bz : wdg_cnt[23];

//------------------------------------------------------------------------------------------------
// Buzzer logic: 0x2800_0001
// Buzz_en: [0] - Open/close buzzer. 0 - close. 1 - open.
//------------------------------------------------------------------------------------------------
`define BUZZER_SUM	25
wire buzz_cs;
reg buzz_en;
reg[`BUZZER_SUM - 1:0] buzz_cnt; 
reg buzz_rst;

assign buzz_cs = !ngcs[MISC_BANK] & (sa[7:0] == 8'b1);

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)            buzz_en <= 1'b0;
	else if(!nwe && buzz_cs)  buzz_en <= sd[0];
end

always @(posedge fpga_clk or negedge sys_rst_n) 
begin
	if(!sys_rst_n) 
		buzz_cnt[`BUZZER_SUM - 1:20] <= 5'b11100;
	else if(!buzz_cnt[`BUZZER_SUM - 1] && buzz_en)
		buzz_cnt[`BUZZER_SUM - 1:20] <= 5'b11100;
	else if(buzz_cnt[`BUZZER_SUM - 1])
		buzz_cnt <= buzz_cnt - `BUZZER_SUM'b1; 
end

assign buzzer_out = buzz_cnt[14] || (!buzz_cnt[`BUZZER_SUM - 1]); 

//------------------------------------------------------------------------------------------------
// IO led out logic: 0x2800_0002
// io_led_out: [0] - io led RUN, default value = 1; 1 - LED is ON, indecate POWER OK, 0 - LED is OFF.
//             [1] - io led ERR, default value = 0. 0 - No error; 1 - Error occure.
//             [2] - io led SIO2 heater, default value = 0. 0 - No heat. 1 - Heater is ON.
//             [3] - io led FILTER heater. default value = 0. 0 - No heat. 1 - Heater is ON.
//             [4] - io led net. Ethernet communication LED.
//             [5] - io led COM. RS485 communication LED.
//------------------------------------------------------------------------------------------------
wire io_led_out_cs;
reg [3:0] io_led_out_reg;

assign io_led_out_cs = !ngcs[MISC_BANK]  & (sa[7:0] == 8'b10);

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)               		io_led_out_reg <= 4'b1;
	else if(!nwe && io_led_out_cs)   io_led_out_reg <= sd[3:0];
end

//assign io_led_out[3:2] = {~io_led_out_reg[3], ~io_led_out_reg[2]};
assign io_led_out[4]   = led_speed ? 1'b1 : ~led_link;
assign io_led_out[5]   = (cpu_txd[1] & cpu_rxd[1] & cpu_txd[2] & cpu_rxd[2]);

//------------------------------------------------------------------------------------------------
// led PWM logic offset: 0x6 ~ 0x8
// led_pwm: [0] - RUN PWM LED enable. 0 - disable. 1 - enable.
//          [1] - ERR PWM LED enable. 0 - disable. 1 - enable.
// led_pwn_cnt: [7:0] - You can set 0 ~ 255
//------------------------------------------------------------------------------------------------
`define PWM_CNT_WIDTH	16
wire led_pwm_en_cs;
wire [1:0] led_pwm_cnt_cs;
reg [1:0] led_pwm_en;
reg [`PWM_CNT_WIDTH - 1:0] led_load_cnt;

assign led_pwm_en_cs     = !ngcs[MISC_BANK] & (sa[7:0] == 8'b110);
assign led_pwm_cnt_cs[0] = !ngcs[MISC_BANK] & (sa[7:0] == 8'b111);
assign led_pwm_cnt_cs[1] = !ngcs[MISC_BANK] & (sa[7:0] == 8'b1000);

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)                  led_pwm_en <= 2'b0;
	else if(!nwe && led_pwm_en_cs)  led_pwm_en <= sd[1:0];
end

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)         		    led_load_cnt <= LED_PWM_CNT_INIT_SUN;
	else if(!nwe && led_pwm_cnt_cs[0])  led_load_cnt[7:0] <= sd;
	else if(!nwe && led_pwm_cnt_cs[1])  led_load_cnt[`PWM_CNT_WIDTH - 1:8] <= sd;
end

//------------------------------------------------------------------------------------------------
reg [2:0] pwm_clk_div;
wire pwm_clk;
reg [`PWM_CNT_WIDTH - 1:0] led_pwm_cnt;
reg led_pwm;
parameter LED_PWM_CNT_INIT_SUN = 16'h8080;

always @(posedge clk_1mhz or negedge sys_rst_n)
begin
	if(!sys_rst_n)    pwm_clk_div <= 3'b0;
	else             pwm_clk_div <= pwm_clk_div + 3'b1;
end

assign pwm_clk = pwm_clk_div[2];

always @(posedge pwm_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)                             led_pwm_cnt <= LED_PWM_CNT_INIT_SUN;
	else if(led_pwm_cnt == `PWM_CNT_WIDTH'b0)  led_pwm_cnt <= led_load_cnt;
	else                                       led_pwm_cnt <= led_pwm_cnt - `PWM_CNT_WIDTH'b1;
end

always @(posedge pwm_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)                             led_pwm = 1'b1;
	else if(led_pwm_cnt == `PWM_CNT_WIDTH'b0) led_pwm = ~led_pwm;  
end

assign io_led_out[0] = led_pwm_en[0] ? led_pwm : (~io_led_out_reg[0]);
assign io_led_out[1] = led_pwm_en[1] ? led_pwm : (~io_led_out_reg[1]);

//------------------------------------------------------------------------------------------------
// Relay output control logic: 0x2800_0003 ~ 0x2800_0004
// relay_en: [0] - Enable SIO2 heater relay output. 0 - Disable; 1 - Enable.
//           [1] - Enable another SIO2 heater relay output. 0 - Disable; 1 - Enable.
//           [2] - Enable FILTER heater relay output. 0 - Disable; 1 - Enable.
//           [3] - Enable CASE heater relay output. 0 - Disable; 1 - Enable.
//             
//	relay_ctrl: [0] - Open/Close 24V DC to start SIO2 heater. 0 - close. 1 - open.
//             [1] - Open/Close 24V DC to start another SIO2 heater. 0 - close. 1 - open.
//             [2] - Open/Close 24V DC to start FILTER heater. 0 - close. 1 - open.
//             [3] - Open/Close 24V DC to start CASE heater. 0 - close. 1 - open.
//------------------------------------------------------------------------------------------------
reg [3:0] relay_enable;
reg [3:0] relay_control;
wire relay_en_cs;
wire relay_ctrl_cs;

assign relay_en_cs   = !ngcs[MISC_BANK] && (sa[7:0] == 8'b11);
assign relay_ctrl_cs = !ngcs[MISC_BANK] && (sa[7:0] == 8'b100);

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)                relay_enable <= 4'b0;
	else if(!nwe && relay_en_cs)  relay_enable <= sd[3:0];
end

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)                  relay_control <= 4'b0;
	else if(!nwe && relay_ctrl_cs)  relay_control <= sd[3:0];
end

assign relay_ctrl[0] = (relay_control[0] & relay_enable[0]);
assign relay_ctrl[1] = (relay_control[1] & relay_enable[1]);
assign relay_ctrl[2] = (relay_control[2] & relay_enable[2]);
assign relay_ctrl[3] = (relay_control[3] & relay_enable[3]);

assign io_led_out[2] = !(io_led_out_reg[2] | relay_ctrl[0] | relay_ctrl[1]);
assign io_led_out[3] = !(io_led_out_reg[3] | relay_ctrl[2] | relay_ctrl[3]);

//---------------------------------------------------------------------------------------------------------------------------------
//	KEY_BOARD logic, 8 bit data bus. 4 x 4 Matrix KEY_BOARD
//	KEY_BOARD address offset: 0x9
//---------------------------------------------------------------------------------------------------------------------------------
`define KEY_CLK_WIDTH	10
wire [`KEY_CLK_WIDTH - 1:0] key_cnt;
wire key_clk;
wire key_cs;
wire key_shift_en;
wire [7:0] key_code;
reg [3:0] key_shift;
wire irq_key;

assign key_cs       = !ngcs[MISC_BANK] && (sa[`SA_ADDR_WIDTH - 1:0] == `SA_ADDR_WIDTH'b1001);
assign key_shift_en = &cpld_key_in;
assign irq_key      = &cpld_key_in;


my_counter C1(.aclr(!sys_rst_n), .clock(clk_1mhz), .q(key_cnt));		//1Mhz / 1024 = 1Khz
assign key_clk = key_cnt[`KEY_CLK_WIDTH - 1];

always @(posedge key_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)         key_shift <= 4'b1110;
	else if(key_shift_en)  key_shift <= {key_shift[0], key_shift[3:1]};
end

assign cpld_key_out = key_shift;
assign key_code     = key_shift_en ? 8'hff : {cpld_key_in[3:0], cpld_key_out};

//---------------------------------------------------------------------------------------------------------------------------------
//	LVDS control logic, 
//	address offset: 0xa
//---------------------------------------------------------------------------------------------------------------------------------
wire lvds_ctrl_cs;
reg [1:0] lvds_ctrl;

assign lvds_ctrl_cs = !ngcs[MISC_BANK] && (sa[`SA_ADDR_WIDTH - 1:0] == `SA_ADDR_WIDTH'b1010);

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)                lvds_ctrl <= 2'b01;
	else if(!nwe && lvds_ctrl_cs) lvds_ctrl <= sd[1:0];
end

assign {lcd_vled, lvds_pwen} = lvds_ctrl;

//------------------------------------------------------------------------------------------------
// Temperature and humidity sensor logic offset: 0x10 ~ 0x15
//------------------------------------------------------------------------------------------------
wire wsd_ctrl_cs;
wire [4:0] wsd_data_cs;
wire [39:0] wsd_data;
wire wsd_data_ready;
wire wsd_state_cs;
wire [2:0] wsd_state;
wire wsd_clk;
reg wsd_ctrl;

assign wsd_ctrl_cs    = !ngcs[MISC_BANK] && (sa[7:0] == 8'b0001_0000);
assign wsd_data_cs[0] = !ngcs[MISC_BANK] && (sa[7:0] == 8'b0001_0001);
assign wsd_data_cs[1] = !ngcs[MISC_BANK] && (sa[7:0] == 8'b0001_0010);
assign wsd_data_cs[2] = !ngcs[MISC_BANK] && (sa[7:0] == 8'b0001_0011);
assign wsd_data_cs[3] = !ngcs[MISC_BANK] && (sa[7:0] == 8'b0001_0100);
assign wsd_data_cs[4] = !ngcs[MISC_BANK] && (sa[7:0] == 8'b0001_0101);
assign wsd_state_cs   = !ngcs[MISC_BANK] && (sa[7:0] == 8'b0001_0110);

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)                wsd_ctrl <= 1'b1;
	else if(!nwe && wsd_ctrl_cs)  wsd_ctrl <= sd[0];
end

assign wsd_sample[1] = wsd_ctrl;
assign wsd_clk       = clk_1mhz;

wsd WSD1
( 
	.wsd_start(wsd_sample[1]), 
	.wsd_clk(wsd_clk), 
	.reset_n(sys_rst_n),       
	.wsd_in(wsd[1]), 
	.q(wsd_data),              
	.data_ready(wsd_data_ready),
	.state(wsd_state)
);

//------------------------------------------------------------------------------------------------
// The 2nd Temperature and humidity sensor logic offset: 0x30 ~ 0x35
//------------------------------------------------------------------------------------------------
wire wsd2_ctrl_cs;
wire [4:0] wsd2_data_cs;
wire [39:0] wsd2_data;
wire wsd2_data_ready;
wire wsd2_state_cs;
wire [2:0] wsd2_state;
reg wsd2_ctrl;

assign wsd2_ctrl_cs    = !ngcs[MISC_BANK] && (sa[`SA_ADDR_WIDTH - 1:0] == `SA_ADDR_WIDTH'b0011_0000);
assign wsd2_data_cs[0] = !ngcs[MISC_BANK] && (sa[`SA_ADDR_WIDTH - 1:0] == `SA_ADDR_WIDTH'b0011_0001);
assign wsd2_data_cs[1] = !ngcs[MISC_BANK] && (sa[`SA_ADDR_WIDTH - 1:0] == `SA_ADDR_WIDTH'b0011_0010);
assign wsd2_data_cs[2] = !ngcs[MISC_BANK] && (sa[`SA_ADDR_WIDTH - 1:0] == `SA_ADDR_WIDTH'b0011_0011);
assign wsd2_data_cs[3] = !ngcs[MISC_BANK] && (sa[`SA_ADDR_WIDTH - 1:0] == `SA_ADDR_WIDTH'b0011_0100);
assign wsd2_data_cs[4] = !ngcs[MISC_BANK] && (sa[`SA_ADDR_WIDTH - 1:0] == `SA_ADDR_WIDTH'b0011_0101);
assign wsd2_state_cs   = !ngcs[MISC_BANK] && (sa[`SA_ADDR_WIDTH - 1:0] == `SA_ADDR_WIDTH'b0011_0110);

always @(posedge fpga_clk or negedge sys_rst_n)
begin
	if(!sys_rst_n)                 wsd2_ctrl <= 1'b1;
	else if(!nwe && wsd2_ctrl_cs)  wsd2_ctrl <= sd[0];
end

assign wsd_sample[2] = wsd2_ctrl;

wsd WSD2
( 
	.wsd_start(wsd_sample[2]), 
	.wsd_clk(wsd_clk), 
	.reset_n(sys_rst_n),       
	.wsd_in(wsd[2]), 
	.q(wsd2_data),             
	.data_ready(wsd2_data_ready),
	.state(wsd2_state)
);

assign cpld_led = {&wsd_sample, wdg_cnt[23]};
assign irq_cpld = {1'b1, irq_key, wsd2_data_ready, wsd_data_ready};

//------------------------------------------------------------------------------------------------
// System ID reg cs: 0x2800_0020 ~ 0x2800_0025
//------------------------------------------------------------------------------------------------
wire [5:0] system_id_cs;

assign system_id_cs[0] = !ngcs[MISC_BANK] & (sa[7:0] == 8'b0010_0000);
assign system_id_cs[1] = !ngcs[MISC_BANK] & (sa[7:0] == 8'b0010_0001);
assign system_id_cs[2] = !ngcs[MISC_BANK] & (sa[7:0] == 8'b0010_0010);
assign system_id_cs[3] = !ngcs[MISC_BANK] & (sa[7:0] == 8'b0010_0011);
assign system_id_cs[4] = !ngcs[MISC_BANK] & (sa[7:0] == 8'b0010_0100);
assign system_id_cs[5] = !ngcs[MISC_BANK] & (sa[7:0] == 8'b0010_0101);


//------------------------------------------------------------------------------------------------
// sd data logic
//------------------------------------------------------------------------------------------------
assign sd = noe ? {8{1'bz}} : ( wdg_cs             ? {6'b0, wdg_pwm, wdg_en} 
                            : ( buzz_cs            ? {7'b0, buzz_en} 									
			    : ( io_led_out_cs      ? {4'b0, io_led_out_reg}
			    : ( led_pwm_en_cs      ? {6'b0, led_pwm_en}
			    : ( led_pwm_cnt_cs[0]  ? led_load_cnt[7:0]
			    : ( led_pwm_cnt_cs[1]  ? led_load_cnt[15:8]
			    : ( relay_en_cs        ? {4'b0, relay_enable}
			    : ( relay_ctrl_cs      ? {4'b0, relay_control}
			    : ( key_cs             ? key_code
			    : ( lvds_ctrl_cs       ? {6'b0, lvds_ctrl}
			    : ( wsd_ctrl_cs        ? {wsd_data_ready, 6'b0, wsd_ctrl}
			    : ( wsd_data_cs[0]     ? wsd_data[39:32]			//humidity high 8 bit
			    : ( wsd_data_cs[1]     ? wsd_data[31:24]			//humidity low 8 bit
			    : ( wsd_data_cs[2]     ? wsd_data[23:16]			//Temperature high 8 bit
			    : ( wsd_data_cs[3]     ? wsd_data[15:8]			//Temperature low 8 bit
			    : ( wsd_data_cs[4]     ? wsd_data[7:0]			//Verify checksum: = humidity_high8 + humidity_low8 + Temperature_high8 + Temperature_low8
			    : ( wsd_state_cs       ? {5'b0, wsd_state}
			    : ( wsd2_ctrl_cs        ? {wsd2_data_ready, 6'b0, wsd2_ctrl}
			    : ( wsd2_data_cs[0]     ? wsd2_data[39:32]			//humidity high 8 bit
			    : ( wsd2_data_cs[1]     ? wsd2_data[31:24]			//humidity low 8 bit
			    : ( wsd2_data_cs[2]     ? wsd2_data[23:16]			//Temperature high 8 bit
			    : ( wsd2_data_cs[3]     ? wsd2_data[15:8]			//Temperature low 8 bit
			    : ( wsd2_data_cs[4]     ? wsd2_data[7:0]			//Verify checksum: = humidity_high8 + humidity_low8 + Temperature_high8 + Temperature_low8
			    : ( wsd2_state_cs       ? {5'b0, wsd2_state}
			    : ( system_id_cs[0]    ? 8'h9a
			    : ( system_id_cs[1]    ? 8'hf4
			    : ( system_id_cs[2]    ? 8'h6c
			    : ( system_id_cs[3]    ? 8'h7f
			    : ( system_id_cs[4]    ? 8'hde
			    : ( system_id_cs[5]    ? 8'h13
			    : {8{1'bz}} ))))))))))))))))))))))))))))));

endmodule
