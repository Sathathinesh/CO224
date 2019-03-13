//   Group 01 
//  E/15/366 and E/15/373

module testbench;

reg clk;
reg reset;
wire W_t,R_d;
wire [7:0] Result,OU1,OU2,D2,mux_1out,im,mux_2out,muxs_out,R_d_data;
wire [3:0] Control;

initial
    
	 $monitor("Res=%b O1=%b O2=%b D2=%b newmux1=%b imm=%b newmux2=%b muxspecial=%b wrt_sig=%b W_t=%b rd_sig=%b R_d=%b S=%b clk=%b R=%b",Result,OU1,OU2,D2,mux_1out,im,mux_2out,muxs_out,W_t,Result,R_d,R_d_data,Control,clk,reset);
	 
	// $strobe("Res=%b O1=%b O2=%b D2=%b newmux1=%b imm=%b newmux2=%b muxspecial=%b R_d=%b S=%b clk=%b R=%b",Result,OU1,OU2,D2,mux_1out,im,mux_2out,muxs_out,R_d_data,Control,clk,reset);
    
    initial
 begin
    $dumpfile("testbench.vcd");
    $dumpvars(0,testbench);
 end
    
proccessor newproc(Result,OU1,OU2,D2,mux_1out,im,mux_2out,muxs_out,R_d_data,R_d,W_t,Control,clk,reset);

initial
begin
	clk=1'b1;
	reset=0;
end

always #5 clk=~clk;

initial
begin
	#120 $finish;
end
endmodule

///////////////////////////////////////////////////////////////////////////////////////////////
module alu(Result,D1,D2,Control);
input [7:0] D1,D2;
input [3:0] Control;
output reg [7:0] Result;

always@(D1,D2,Control)
begin
case(Control)
3'b000:Result<=D1;
3'b001:Result<=D1+D2;
3'b010:Result<=D1&D2;
3'b011:Result<=D1|D2;
3'b100:Result<=D1;
3'b101:Result<=8'dx;
endcase
end
endmodule

///////////////////////////////////////////////////////////////////////////////////////////////
module regfile8x8a(OU1,OU2,clk,INaddr,OU1addr,OU2addr,IN);

input clk;
input [7:0] IN;
output [7:0] OU1,OU2;
input [2:0] INaddr,OU1addr,OU2addr;
reg [7:0] reg0, reg1, reg2, reg3,reg4,reg5,reg6,reg7;

assign OU1 = (OU1addr==3'b000)?reg0:
(OU1addr==3'b001)?reg1:
(OU1addr==3'b010)?reg2:
(OU1addr==3'b011)?reg3:
(OU1addr==3'b100)?reg4:
(OU1addr==3'b101)?reg5:
(OU1addr==3'b110)?reg6:
(OU1addr==3'b111)?reg7:0;
// add until 8 //
assign OU2 = OU2addr == 0 ? reg0 :
OU2addr == 1 ? reg1 :
OU2addr == 2 ? reg2 :
OU2addr == 3 ? reg3 :
OU2addr == 4 ? reg4 :
OU2addr == 5 ? reg5 :
OU2addr == 6 ? reg6 :
OU2addr == 7 ? reg7 :0;
//add until 8//
always @(negedge clk) 
begin
case(INaddr)
3'b000:reg0=IN;
3'b001:reg1=IN;
3'b010:reg2=IN;
3'b011:reg3=IN;
3'b100:reg4=IN;
3'b101:reg5=IN;
3'b110:reg6=IN;
3'b111:reg7=IN;
// your code here
endcase
end // always @ (negedgeclk)
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module CU(OU1addr,OU2addr,INaddr,busy_signal,R_d,W_t,Address,im,Control,imm_signal,comp_signal,inst,busy_wait);

reg busy;
input busy_wait;
input [31:0] inst;
output reg [7:0] im;
output reg imm_signal;
output reg [3:0] Control;
output reg [2:0] OU1addr;
output reg [2:0] OU2addr;
output reg [2:0] INaddr;
output reg [7:0] Address;
output reg R_d;
output reg W_t;
output reg busy_signal=1'b1;
output reg comp_signal;
always @(inst) 
begin
	imm_signal = 1'b0;
	comp_signal = 1'b0;
	R_d = 1'b0;
	W_t = 1'b0;
	busy = 1'b1;
	Control = inst[26:24];
	case(inst[31:24])
		8'b00000100:
		begin
			Address = inst[23:16];
			imm_signal = 1'b0;
			W_t = 1;
			R_d = 0;
			OU2addr = inst[2:0];
			busy = 1'b0;
		end	
		8'b00000101:
		begin
			Address = inst[7:0];
			INaddr = inst[18:16];
			W_t = 0;
			R_d = 1;
			busy = 1'b0;			
		end		
		default:
            busy=1'b1;
	endcase		
	if (busy)
	begin
		im = inst[7:0];
		Control = inst[26:24];
		case (inst[31:24])
			8'b00001000:
				imm_signal = 1'b1;
			8'b00001001:						
				comp_signal = 1'b1;		
			default:;
		endcase
		if (busy)
		begin
			INaddr = inst[18:16]; //& busy_wait;
			OU2addr = inst[2:0]; //& busy_wait;
			OU1addr = inst[10:8]; //& busy_wait;
		end
	end
	busy_signal = busy;
end
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////
module mux(out,Control,input1,input2);

input Control,clk;
input [7:0] input1,input2;
output reg [7:0] out;
always @* begin
	if (Control==1) 
		out = input1;
	else 
		out = input2;
end
endmodule
///////////////////////////////////////////////////////////////////////////////////////////////////////
module newMux(out,Control,input1,input2);

input Control,clk;
input [7:0] input1,input2;
output reg [7:0] out;
always @* begin
	if (Control==0) 
		 out = input1;
	else 
		 out = input2;
end
endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////


module reginsts(inst,clk,R_d_Addr,busy_signal);
input clk,busy_signal;
input [3:0] R_d_Addr;
output reg [31:0] inst;
reg reg_clock=1'b1;

reg [31:0] addr1 = 32'b00001000000001000000000011111111;		// loadi 4, X, 0xFF
reg [31:0] addr2 = 32'b00001000000001100000000010101010;		// loadi 6, X, 0xAA
reg [31:0] addr3 = 32'b00001000000000110000000010111011;		// loadi 3, X, 0xBB
reg [31:0] addr4 = 32'b00000001000001010000011000000011;		// add   5, 6, 3
reg [31:0] addr5 = 32'b00000010000000010000010000000101;		// and   1, 4, 5
reg [31:0] addr6 = 32'b00000011000000100000000100000110;		// or    2, 1, 6 
reg [31:0] addr7 = 32'b00000000000001110000000000000010;		// mov   7, x, 2
reg [31:0] addr8 = 32'b00001001000001000000011100000011;		// sub   4, 7, 3
reg [31:0] addr9 = 32'b00000100111111110000000000000100;		// store 4, X, 0xFF
reg [31:0] addr10 = 32'b00000101000001100000000011111111;		// load 6, X, 0xFF
reg [31:0] addr11 = 32'b00001001000001000000011100000110;		// sub   4, 7, 6
reg [31:0] addr12 = 32'b00000000000000000000000000000000;		// do nothing

always @(negedge clk) 
begin
  //  if(!busy_signal)
 //   begin
 //       repeat (1)
 //        reg_clock= !reg_clock;	
 //   end    
	case (R_d_Addr)
		4'd0:inst = addr1;
		4'd1:inst = addr2;
		4'd2:inst = addr3;
		4'd3:inst = addr4;
		4'd4:inst = addr5;
		4'd5:inst = addr6;
		4'd6:inst = addr7;
		4'd7:inst = addr8;
		4'd8:inst = addr9;
        4'd9:inst = addr12;
		4'd10:inst = addr10;
		4'd11:inst = addr11;
		
		default :;
	endcase
end
endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////////////
module counter (R_d_addr,busy_signal,clk,reset);

input clk,busy_signal;
input reset;
output reg [3:0] R_d_addr=0;
always @(negedge clk)

if(!reset) 
begin
	R_d_addr<=R_d_addr+4'd1;
end
else 
begin
	R_d_addr<=4'd0;	
end
endmodule

//////////////////////////////////////////////////////////////////////////////////////////////////////

module compliment(out,in);
output [7:0] out;
input [7:0] in;
reg [7:0] comp=8'b11111111;
assign out=(comp-in)+8'b00000001;
endmodule


/////////////////////////////////////////////////////////////////////////////////////////////////////////

module data_mem(R_d_data,busy_wait,W_t_data,R_d,W_t,address,clk,reset);

input clk;
input reset;
input R_d;
input W_t;
input[7:0] address;
input[7:0] W_t_data;
output reg [7:0] R_d_data;
output reg busy_wait;
reg clock =1'b1;
integer  i;


// Declare memory 256x8 bits 
reg [7:0] memory_array [255:0];
//reg [7:0] memory_ram_r [255:0];


always @(negedge reset)
begin
    if (reset)
    begin
        for (i=0;i<256; i=i+1)
            memory_array[i] <= 0;
    end
end

always @(*)
begin
    if (W_t && !R_d)
	begin
		//busy_wait <= 0;
        // artificially delay 100 cycles	
		memory_array[address] = W_t_data;
        repeat (5)
		#1 clock= !clock;
		//busy_wait <= 1;
	end
    if (!W_t && R_d)
	begin
		//busy_wait <= 0;
		// artificially delay 100 cycles
		R_d_data = memory_array[address];
		repeat (5)
        #1 clock= !clock;
		//busy_wait <= 1;
	end
	if(!R_d && !W_t)
	begin
        R_d_data=8'bx;
    end    
end
 
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////

module proccessor(Result,OU1,OU2,D2,mux_1out,im,mux_2out,newMuxout,R_d_data,R_d,W_t,Control,clk,reset);

input reset;
input clk;
output [3:0] Control;
output [7:0] im,OU2,D2,mux_1out,Result,OU1,mux_2out,newMuxout,R_d_data;
output R_d,W_t;
wire [7:0] Address;
wire [3:0] R_d_addr;
wire [31:0] inst;
wire busy_wait;
wire [2:0] OU1addr,OU2addr,INaddr;
wire comp_signal,imm_signal,busy_signal;

counter newcounter(R_d_addr,busy_signal,clk,reset);
reginsts newreg(inst,clk,R_d_addr,busy_signal);
CU newcu(OU1addr,OU2addr,INaddr,busy_signal,R_d,W_t,Address,im,Control,imm_signal,comp_signal,inst,busy_wait);
data_mem mydata_mem1(R_d_data,busy_wait,Result,R_d,W_t,Address,clk,reset);
regfile8x8a newregister(OU1,OU2,clk,INaddr,OU1addr,OU2addr,newMuxout);
compliment newcomp(D2,OU2);
mux newmux1(mux_1out,comp_signal,D2,OU2);
mux newmux2(mux_2out,imm_signal,im,mux_1out);
alu newalu(Result,mux_2out,OU1,Control);
newMux muxnew(newMuxout,busy_signal,R_d_data,Result);


endmodule