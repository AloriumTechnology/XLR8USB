// HOST_USB
// Creator: Ted Holler
// Date: 1/2/2018
// HOST USB is used to interface to a USB FULL SPEED MOUSE device
// This module will send Packet to the USB Mouse and determine which
// bits in the Endpoint #2 will be used for mouse data.
// 
// This code currently works on these Mice.
// Logitech M325, M325U, M187

//
//                     +----------------------+
//      USB connector  |  VCC  D-   D+   GND  |
//                     +--***--***--***--***--+
//                        Red  Yel  Whi  Black  <<==usb snooper connector
//                             Brn  Blk         <<==Logic Analyzer wires
//                             A5   A4


module host_usb(clk,  //16Mhz CPU clock
                clk60, //60Mhz USB clock
                rst_n,
                dp_i,
                dm_i,
                d__oen,
                dp_o,
                dm_o,
                ramadr,
                ramre,
                ramwe,
                dm_sel,
                dbus_in,
                dbus_out,
                dbus_out_en
               );

parameter USB_CNTL_ADDR = 8'hE0;
parameter USB_BTNS_ADDR = 8'hE1;
parameter USB_XMOV_ADDR = 8'hE2;
parameter USB_YMOV_ADDR = 8'hE3;
parameter USB_SCRL_ADDR = 8'hE4;
parameter WIDTH = 8;



// 25Mhz/16 = 1.56Mhz  (Low speed USB is 1.5Mhz,  Full speed is 12Mhz)
// 50Mhz/32 = 1.56Mhz  (Low speed USB is 1.5Mhz,  Full speed is 12Mhz)
parameter MAX_CYCLES=4;    //5 clocks / bit period


//Tokens
parameter WRITE  =4'b0001; //OUT
parameter READ   =4'b1001; //IN
parameter SOF    =4'b0101;
parameter SETUP  =4'b1101;
//Data
parameter DATA0  =4'b0011;
parameter DATA1  =4'b1011;
//Handshake
parameter ACK    =4'b0010;
parameter NACK   =4'b1010;
parameter STALL  =4'b0110;

parameter WAIT_CMD_DONE = 7'h7F; //127
parameter RESTART       = 7'h7E; //126
parameter GO_DLY        = 7'h7D; //125

reg r_ack;
reg r_nack;
reg r_stall;
reg mouse_data;
reg mdata_cap;
reg[4:0] mdata;

reg[7:0]  btns;
reg[15:0] x_move;
reg[15:0] y_move;
reg[7:0]  scroll;

wire  idle;
reg   idle1;
reg   idle2;
wire  dout_en;
wire  dp_o;
wire  dm_o;
reg   set_addr;
reg   stat;
reg   full_speed;
reg   rst_active;
reg   usb_rdy;

logic     ctrl_sel;
logic     ctrl_we;
logic     ctrl_re;
logic [WIDTH-1:0] usb_ctrl; 
logic     btns_sel;  //------ BTNS (2^0 = left, 2^1 = right, 2^2 = middle)
logic     btns_we;
logic     btns_re;
logic [WIDTH-1:0] usb_btns;
logic     xmov_sel;  //------  X movement
logic     xmov_we;
logic     xmov_re;
logic [WIDTH-1:0] usb_xmov;
logic     ymov_sel;  //------- Y movement
logic     ymov_we;
logic     ymov_re;
logic [WIDTH-1:0] usb_ymov;
logic     scrl_sel;  //------- SCROLL wheel
logic     scrl_we;
logic     scrl_re;
logic [WIDTH-1:0] usb_scrl;

reg [63:0] snd_data;
//reg [31:0] tout_cnt;
reg [31:0] rsend_cnt;
reg [31:0] start_time;
reg [19:0] read_time;
reg [15:0] timeout_cnt;
reg [6:0]  state;
reg [6:0]  nxt_state;
reg [3:0]  pid;
reg [6:0]  addr;
reg [3:0]  end_pt;
reg        snd;
reg        dev_rst;
reg        device;    // 0=M325c,M187  1=M325
reg [15:0] hld_off;
reg [2:0]  btn_dly;
reg        d_v_r;
reg        d_v_r_dly;

reg  host_bsy;
reg  host_bsy1;

reg[3:0]  md_16;  //mouse_data resync
reg[7:0]  ctrl_16;
reg[7:0]  x_move_16;
reg[7:0]  y_move_16;
reg[7:0]  scroll_16;
reg[7:0]  btns_16;
reg       ctrl_usb_rst;
reg       x_rd_dly,x_rd_dly1,x_rd_dly2;
reg       y_rd_dly,y_rd_dly1,y_rd_dly2;
reg       s_rd_dly,s_rd_dly1,s_rd_dly2;

wire    chip_rst_n = rst_n & ~ctrl_usb_rst;

wire       rcv_dp,rcv_dm;
wire       cmd_done;
wire[63:0] data_out /* synthesis keep */;
wire       d_v      /* synthesis keep */;
wire       nack;
wire       ack;
wire       stall;
wire       err;
wire       usb_rst;

  // I/O 
   input   clk;  //16Mhz  
   input   clk60;
   input   rst_n;

   //USB signals
   output  dp_o;
   output  dm_o;
   output  d__oen;
   input   dp_i;
   input   dm_i;

   input  [7:0]              dbus_in;
   output [7:0]              dbus_out;
   output                    dbus_out_en;
   // DM
   input [7:0]               ramadr;
   input                     ramre;
   input                     ramwe;
   input                     dm_sel;

//---------------------------------------------------------------------------
//------   16Mhz Domain -----------------------------------------------------
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv


 always @(posedge clk) begin
    if(~rst_n) begin
       md_16 <= 4'h0;
    end else begin
       md_16[3:0] <= {md_16[2:0], mouse_data};
    end
 end

 wire sample_mouse = (md_16[3]==0 && md_16[2]==1);

 always @(posedge clk) begin
    if(~rst_n) begin
       ctrl_16   <= 8'h00;
       btns_16   <= 8'h00;
       x_move_16 <= 8'h00;
       y_move_16 <= 8'h00;
       scroll_16 <= 8'h00;
    end else begin
       ctrl_16   <= {usb_rdy, 6'h00,  ctrl_usb_rst };
       btns_16   <= (sample_mouse==1)  ?  {5'h0, btns[2:0]} : btns_16;
       x_move_16 <= (x_rd_dly2==1) ? 8'h00 : (sample_mouse==1) ? x_move[7:0] : x_move_16;
       y_move_16 <= (y_rd_dly2==1) ? 8'h00 : (sample_mouse==1) ? y_move[7:0] : y_move_16;
       scroll_16 <= (s_rd_dly2==1) ? 8'h00 : (sample_mouse==1) ? scroll[7:0] : scroll_16;
    end
 end

 always @(posedge clk) begin
    if(~rst_n) begin
       ctrl_usb_rst <=0; 
    end else begin
       if(ctrl_we==1) ctrl_usb_rst <= dbus_in[0];
    end
 end

 assign dbus_out = ({8{ctrl_sel}} & ctrl_16) |
                   ({8{btns_sel}} & btns_16) |
                   ({8{xmov_sel}} & x_move_16) |
                   ({8{ymov_sel}} & y_move_16) |
                   ({8{scrl_sel}} & scroll_16); 

 assign io_out_en = ctrl_re ||
                    btns_re ||
                    xmov_re ||
                    ymov_re ||
                    scrl_re;



assign A3 = (timeout_cnt == 0);
assign A2 = ctrl_usb_rst;
assign A1 = chip_rst_n;

assign dbus_out_en = ctrl_re ||
                     btns_re ||
                     xmov_re ||
                     ymov_re ||
                     scrl_re;

assign  ctrl_sel = (dm_sel && ramadr == USB_CNTL_ADDR );
assign  ctrl_we  = ctrl_sel && ramwe;
assign  ctrl_re  = ctrl_sel && ramre;
assign  btns_sel = (dm_sel && ramadr == USB_BTNS_ADDR );
assign  btns_we  = btns_sel && ramwe;
assign  btns_re  = btns_sel && ramre;
assign  xmov_sel = (dm_sel && ramadr == USB_XMOV_ADDR );
assign  xmov_we  = xmov_sel && ramwe;
assign  xmov_re  = xmov_sel && ramre;
assign  ymov_sel = (dm_sel && ramadr == USB_YMOV_ADDR );
assign  ymov_we  = ymov_sel && ramwe;
assign  ymov_re  = ymov_sel && ramre;
assign  scrl_sel = (dm_sel && ramadr == USB_SCRL_ADDR );
assign  scrl_we  = scrl_sel && ramwe;
assign  scrl_re  = scrl_sel && ramre;

always @(posedge clk) begin
   x_rd_dly2 <= x_rd_dly1;
   x_rd_dly1 <= x_rd_dly;
   x_rd_dly  <= xmov_re;

   y_rd_dly2 <= y_rd_dly1;
   y_rd_dly1 <= y_rd_dly;
   y_rd_dly  <= ymov_re;

   s_rd_dly2 <= s_rd_dly1;
   s_rd_dly1 <= s_rd_dly;
   s_rd_dly  <= scrl_re;
end



//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//------   16Mhz Domain -----------------------------------------------------
//---------------------------------------------------------------------------




always @(posedge clk60) begin
   if(~chip_rst_n || (idle2==0 && idle1==1) ) begin
      host_bsy <=0;
   end else begin
      if(snd) begin
         host_bsy <= 1'b1;
      end
   end
   host_bsy1 <= host_bsy;
end


always @(posedge clk60) begin
   if(~chip_rst_n) begin
      start_time <= 0;
      read_time <= 0;
   end else begin
      //start_time <= (start_time== 59999999 ) ? 0 : start_time+1;
      start_time <= (start_time== 599999 ) ? 0 : start_time+1;
      read_time  <= (read_time == 599999) ? 0 : read_time+1;
   end
end

always @(posedge clk60) begin
   idle2 <= idle1;
   idle1 <= idle;
end


reg  host_done;
reg  host_done_dly;
always @(posedge clk60) begin
   if(~chip_rst_n  || (usb_rst && !rst_active) ) begin
      host_done <= 1;
      host_done_dly <= 0;
   end else begin
      host_done_dly <= host_done;
      host_done <= snd==1 ? 0 : cmd_done==1 ? 1 : host_done;
   end
end

always @(posedge clk60) begin
   mouse_data <= mdata[4];
   mdata[4:0] <= {mdata[3:0],mdata_cap}; 
end

always @(posedge clk60) begin
   if(~chip_rst_n ) begin
      r_ack <= 0;
      r_nack <= 0;
      r_stall <=0;
   end else begin
      if(ack) r_ack <= ~r_ack;
      if(nack) r_nack <= ~r_nack;
      if(stall) r_stall <= ~r_stall;
   end
end

wire timeout;
assign timeout = (timeout_cnt == 0) ? 1:0 ;


always @(posedge clk60) begin
   //if(~chip_rst_n  || (usb_rst && !rst_active) ) begin
   if(~chip_rst_n ) begin
      state <= 0;
      pid <= 0;
      addr <= 0;
      end_pt <= 0;
      snd <= 0;
      nxt_state <= 0;
      snd_data <= 0;
      dev_rst <=0;
      set_addr <=0;
      stat <= 0;
      full_speed <= 1;
      rst_active <= 0;
      mdata_cap <= 0;
      device <= 0; // <<<<---------------- DEVICE
      x_move <= 16'h0;
      y_move <= 16'h0;
      scroll <= 8'h0;
      usb_rdy <=0;
      timeout_cnt <= 0;
   end else begin

      addr   <= 0;
      end_pt <= 0;
      snd    <= 0;
      snd_data <= 0;
      dev_rst <=0;
      set_addr <=0;
      stat <= 0;

      timeout_cnt <= timeout_cnt != 0 ? timeout_cnt -1 : 0;

      case(state) 
         0: begin
               timeout_cnt <= 0;
               usb_rdy <=0;
               rst_active <= 0;
               //state <= (start_time==59000000  ) ? 1 : 0;  //DEbug only
               state <= (start_time==590000  ) ? 1 : 0; //10msec
               //tout_cnt <=0;
            end

     //Initial RESET
         1: begin snd <= 1; dev_rst <= 1 ; addr <= 0; end_pt <= 0; 
               usb_rdy <=0;
               rst_active <= 1;
               state <= WAIT_CMD_DONE; 
               nxt_state <= 2;
            end

         2: begin //Delay a bit
               rst_active <= 0;
               full_speed <=1;
               rsend_cnt <= 480;
               state <= GO_DLY; 
               nxt_state <= 3;
            end

      //Setup to get the Descriptor
      //----------------------------------
         3: begin snd <= 1; pid <= SETUP ; addr <= 0; end_pt <= 0; state <= WAIT_CMD_DONE; 
               rst_active <= 0;
               state <= WAIT_CMD_DONE;  //wait cmd done
               nxt_state <= 4;
            end

         4: begin
                rst_active <= 0;
                snd <= 1;
                pid <= DATA0;
                snd_data <= 64'h0040_0000_0100_0680;            
                state <= WAIT_CMD_DONE;  //wait cmd done
                nxt_state <= 5;
                timeout_cnt <= 1200;
             end

         5: begin //wait here until cmd is ACKed or NACKed
               rst_active <= 0;
               if(nack || stall || timeout) begin
                  //rsend_cnt <= 7200;
                  rsend_cnt <= 1600;
                  state <= GO_DLY; //Delay X cycles
                  nxt_state <= 2; //Try sending Setup to get Descriptor again
               end 
               if(ack) begin
                  //rsend_cnt <= 300;
                  rsend_cnt <= 360;
                  state <= GO_DLY; 
                  nxt_state <= 6;
               end 
            end

         6: begin 
                rst_active <= 0;
                snd <= 1; pid <= WRITE ; addr <= 0; end_pt <= 0; 
                state <= WAIT_CMD_DONE; 
                nxt_state <= 7; 
            end
   
         7: begin
                rst_active <= 0;
                //WRITE STAT to device *************************************
                //        vvvvvvvvv
                snd <= 1; stat <= 1;pid <= DATA0 ; addr <= 0; end_pt <= 0; 
                state <= 8; 
                timeout_cnt <= 600;
            end

        8: begin //wait here until cmd is ACKed or NACKed
               rst_active <= 0;
               if(nack || timeout ) begin
                  //rsend_cnt <= 7200;
                  rsend_cnt <= 1600;
                  state <= GO_DLY; //Delay X cycles
                  nxt_state <= 6; //Try resending Write STAT
               end 
               if(stall) begin
                  //rsend_cnt <= 7200;
                  rsend_cnt <= 1600;
                  state <= GO_DLY; //Delay X cycles
                  nxt_state <= 3; //Try resending Write STAT
               end 
               if(ack) begin
                  //rsend_cnt <= 300;
                  rsend_cnt <= 360;
                  state <= GO_DLY; 
                  nxt_state <= 9;
               end 
            end


        9: begin //Delay a bit
               rst_active <= 0;
               rsend_cnt <= 12000;
               state <= GO_DLY; 
               nxt_state <= 10;
            end



     //Reset the Device so it can be configured ---------------------------------------------------------------
     //
        10: begin  //Send RESET to device
               rst_active <= 1; 
               snd <= 1; dev_rst <=1 ; addr <= 0; end_pt <= 0; 
               state <= WAIT_CMD_DONE; 
               nxt_state <= 11; 
            end

        11: begin //Delay a bit
               rst_active <= 0;
               //rsend_cnt <= 12000;
               rsend_cnt <= 82000;
               state <= GO_DLY; 
               nxt_state <= 12;
            end

//********* 12 ---> 21  *****************************************************************
//*********   SET the ADDRESS for the DEVICE to 6 ***************************************
//***************************************************************************************
        12: begin 
               rst_active <= 0;
               snd <= 1; pid <= SETUP ; addr <= 0; end_pt <= 0; 
               state <= WAIT_CMD_DONE; 
               nxt_state <= 13; 
            end

        // Data word  to set device addr to 6
        13: begin
                rst_active <= 0;
                snd <= 1;
                pid <= DATA0;
                snd_data <= 64'h0000_0000_0006_0500;  //Send Device ADDR=6
                set_addr <= 1;
                state <= WAIT_CMD_DONE;
                nxt_state <= 15; //wait 2 ack
                timeout_cnt <= 1200;
             end

        15: begin
               rst_active <= 0;
               if(nack || stall || timeout) begin
                  //rsend_cnt <= 7200;
                  rsend_cnt <= 1600;
                  state <= GO_DLY; //Delay X cycles
                  nxt_state <= 11; //Try sending Setup to get Descriptor again
               end 
               if(ack) begin
                  //rsend_cnt <= 300;
                  rsend_cnt <= 60;
                  state <= GO_DLY; 
                  nxt_state <= 17;
               end 
            end


         //Request Status
         17: begin 
                rst_active <= 0;
                snd <= 1; pid <= READ ; addr <= 0; end_pt <= 0; 
                state <= 18; 
             end

         18: begin
                rst_active <= 0;
                if(d_v) begin  //NEED to CHECK Data value
                   rsend_cnt <= 10;
                   state <= GO_DLY; 
                   nxt_state <= 20;
                end
             end


         //send a ACK to acknowledge the status
         20: begin 
                rst_active <= 0;
                snd <= 1; pid <= ACK ; addr <= 0; end_pt <= 0; 
                state <= WAIT_CMD_DONE; 
                nxt_state <= 21; 
             end

         //****** Allow 2msec for the device to switch address
         21: begin //Delay a bit
                rst_active <= 0;
                //rsend_cnt <= 500;
                rsend_cnt <= 640000; //  600 was  10usec   60000 = 1msec)
                state <= GO_DLY; 
                nxt_state <= 25;
             end
//^^^^^^^^^^^^^^^^^^^^^^^^^^ SET ADDRESS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//**************************************************************************

 //Setup to get the Descriptor
      //----------------------------------
         25: begin 
                rst_active <= 0;
                snd <= 1; pid <= SETUP ; addr <= 6; end_pt <= 0; 
                state <= WAIT_CMD_DONE; 
                nxt_state <= 26; 
             end

      //Request to get the Descriptor     !!!!!!! Need to see which device we are talking to
      //----------------------------------
         26: begin
               rst_active <= 0;
               snd <= 1;
               pid <= DATA0;
               snd_data <= 64'h0012_0000_0100_0680;            //12   (orginally was 40 low speed ??? )
               state <= WAIT_CMD_DONE; 
               nxt_state <= 27; 
               timeout_cnt <= 1200;
            end

         27: begin
               rst_active <= 0;
               if(nack || stall || timeout) begin
                  //rsend_cnt <= 7200;
                  rsend_cnt <= 1600;
                  state <= GO_DLY; //Delay X cycles
                  nxt_state <= 25; //Try sending Setup to get Descriptor again
               end 
               if(ack) begin
                  //rsend_cnt <= 300;
                  rsend_cnt <= 360;
                  state <= GO_DLY; 
                  nxt_state <= 29;
               end 
            end


//DELAY *************************************************  DELAY  NEEDED
        29: begin //Delay a bit
               rst_active <= 0;
               //rsend_cnt <= 24000;
               rsend_cnt <= 28800;
               state <= GO_DLY; 
               nxt_state <= 30;
            end

//Setup     SET_CONFIG   
      //----------------------------------
         30: begin 
               rst_active <= 0;
               snd <= 1; pid <= SETUP ; addr <= 6; end_pt <= 0;
               state <= WAIT_CMD_DONE; 
               nxt_state <= 31; 
            end


      // 0x09  set_configuration
      //----------------------------------
         31: begin
               rst_active <= 0;
               snd <= 1;
               pid <= DATA0;
               snd_data <= 64'h0000_0000_0001_0900;              //SET_CONFIG 
               state <= WAIT_CMD_DONE; 
               nxt_state <= 32;  
               timeout_cnt <=1200;
            end


        32: begin
               rst_active <= 0;
               if(nack || stall || timeout) begin
                  //rsend_cnt <= 7200;
                  rsend_cnt <= 1600;
                  state <= GO_DLY; //Delay X cycles
                  nxt_state <= 30; //Try sending Setup to get Descriptor again
               end 
               if(ack) begin
                  rsend_cnt <= 360;
                  state <= GO_DLY; 
                  nxt_state <= 34;
               end 
            end

         34: begin 
               rst_active <= 0;
               snd <= 1; pid <= READ ; addr <= 6; end_pt <= 0; 
               state <= WAIT_CMD_DONE; 
               nxt_state <= 35; 
               timeout_cnt <= 1200;
            end

         //  Wait for "STATUS" to be returned......
         35: begin
               rst_active <= 0;
               if(nack || timeout) begin
                  //rsend_cnt <= 7200;
                  rsend_cnt <= 1600;
                  state <= GO_DLY; //Delay X cycles
                  nxt_state <= 34; //Try sending READ again
               end 
               if(stall) begin
                  //rsend_cnt <= 7200;
                  rsend_cnt <= 1600;
                  state <= GO_DLY; //Delay X cycles
                  nxt_state <= 30; //Try sending READ again
               end 
               if(d_v) begin  //NEED to CHECK Data value  //May need to use ACK?????
                  rsend_cnt <= 10;
                  state <= GO_DLY; 
                  nxt_state <= 37;
               end
            end


         // Acknowledge the "STATUS" 
         37: begin 
                rst_active <= 0;
               snd <= 1; pid <= ACK ; addr <= 0; end_pt <= 0; 
               state <= WAIT_CMD_DONE; 
               nxt_state <= 38; 
            end
         //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

         38: begin
               rst_active <= 0;
               rsend_cnt <= 600;
               state <= GO_DLY; 
//                 nxt_state <= 100;
               nxt_state <= 40;
            end


// Get the Device information
         40: begin 
                rst_active <= 0;
                snd <= 1; pid <= SETUP ; addr <= 6; end_pt <= 0; 
                state <= WAIT_CMD_DONE; 
                nxt_state <= 41; 
             end

         41: begin
               rst_active <= 0;
               snd <= 1;
               pid <= DATA0;
               snd_data <= 64'h0012_0000_0100_0680;            //12   (orginally was 40 low speed ??? )
               state <= WAIT_CMD_DONE; 
               nxt_state <= 42; 
               timeout_cnt <=1200;
            end

         42: begin
               rst_active <= 0;
               if(nack || stall || timeout) begin
                  //rsend_cnt <= 7200;
                  rsend_cnt <= 1600;
                  state <= GO_DLY; //Delay X cycles
                  nxt_state <= 40; //Try sending Setup to get Descriptor again
               end 
               if(ack) begin
                  //rsend_cnt <= 300;
                  rsend_cnt <= 360;
                  state <= GO_DLY; 
                  nxt_state <= 50;
               end 
            end

         // Get the USB revision info 
         50: begin 
                rst_active <= 0;
                snd <= 1; pid <= READ ; addr <= 6; end_pt <= 0; 
                state <= WAIT_CMD_DONE; 
                state <= 51; 
                timeout_cnt <= 1200;
             end

         51: begin
                rst_active <= 0;
                if(nack || timeout) begin
                   //rsend_cnt <= 7200;
                   rsend_cnt <= 1600;
                   state <= GO_DLY; //Delay X cycles
                   nxt_state <= 50; //Try sending Setup to get Descriptor again
                end 
                if(stall) begin
                   //rsend_cnt <= 7200;
                   rsend_cnt <= 1600;
                   state <= GO_DLY; //Delay X cycles
                   nxt_state <= 40; //Try sending Setup to get Descriptor again
                end 
                if(d_v) begin  //NEED to CHECK Data value  
                   rsend_cnt <= 10;
                   state <= GO_DLY; 
                   nxt_state <= 52;
                end
             end
         
         52: begin 
                rst_active <= 0;
               snd <= 1; pid <= ACK ; addr <= 0; end_pt <= 0; 
               state <= WAIT_CMD_DONE; 
               nxt_state <= 53; 
             end

         53: begin
                rst_active <= 0;
               rsend_cnt <= 360;
               state <= GO_DLY; 
               nxt_state <= 60;
            end

        // Get the Vendor information *******************************************************
        // **********************************************************************************
         60: begin 
                rst_active <= 0;
                snd <= 1; pid <= READ ; addr <= 6; end_pt <= 0; 
                state <= WAIT_CMD_DONE; 
                state <= 61; 
                timeout_cnt <= 1200;
             end

         61: begin
                rst_active <= 0;
                if(nack || timeout) begin
                   //rsend_cnt <= 7200;
                   rsend_cnt <= 1600;
                   state <= GO_DLY; //Delay X cycles
                   nxt_state <= 60; //Try sending Setup to get Descriptor again
                end 
                if(stall) begin
                   //rsend_cnt <= 7200;
                   rsend_cnt <= 1600;
                   state <= GO_DLY; //Delay X cycles
                   nxt_state <= 40; //Try sending Setup to get Descriptor again
                end 
                if(d_v) begin  //NEED to CHECK Data value  
                   rsend_cnt <= 10;
                   state <= GO_DLY; 
                   nxt_state <= 62;
                   if(data_out[47:32]==16'h1201) begin
                      device <= 1;  //M325
                   end else begin
                      device <= 0;  //M325c or M187
                   end
                end
             end
         
         62: begin 
                rst_active <= 0;
               snd <= 1; pid <= ACK ; addr <= 0; end_pt <= 0; 
               state <= WAIT_CMD_DONE; 
               nxt_state <= 63; 
             end

         63: begin
                rst_active <= 0;
               rsend_cnt <= 360;
               state <= GO_DLY; 
               nxt_state <= 70;
            end

        // Finish readin the rest of the data  (2 bytes )************************************
        // **********************************************************************************
         70: begin 
                rst_active <= 0;
                snd <= 1; pid <= READ ; addr <= 6; end_pt <= 0; 
                state <= WAIT_CMD_DONE; 
                state <= 71; 
                timeout_cnt <=1200;
             end

         71: begin
                rst_active <= 0;
                if(nack || timeout) begin
                   //rsend_cnt <= 7200;
                   rsend_cnt <= 1600;
                   state <= GO_DLY; //Delay X cycles
                   nxt_state <= 70; //Try sending Setup to get Descriptor again
                end 
                if(stall) begin
                   //rsend_cnt <= 7200;
                   rsend_cnt <= 1600;
                   state <= GO_DLY; //Delay X cycles
                   nxt_state <= 40; //Try sending Setup to get Descriptor again
                end 
                if(d_v) begin  //NEED to CHECK Data value  
                   rsend_cnt <= 10;
                   state <= GO_DLY; 
                   nxt_state <= 72;
                end
             end
         
         72: begin 
                rst_active <= 0;
               snd <= 1; pid <= ACK ; addr <= 0; end_pt <= 0; 
               state <= WAIT_CMD_DONE; 
               nxt_state <= 73; 
             end

         73: begin
                rst_active <= 0;
               rsend_cnt <= 60;
               state <= GO_DLY; 
               nxt_state <= 80;
            end

         //Send STATUS to complete the above references
         80: begin 
                rst_active <= 0;
                snd <= 1; pid <= WRITE ; addr <= 6; end_pt <= 0; 
                state <= WAIT_CMD_DONE; 
                nxt_state <= 81; 
             end
   
         81: begin
                rst_active <= 0;
                //WRITE STAT to device *************************************
                //        vvvvvvvvv
                snd <= 1; stat <= 1;pid <= DATA0 ; addr <= 0; end_pt <= 0; 
                state <= 82; 
                timeout_cnt <=600;
             end

         82: begin //wait here until cmd is ACKed or NACKed
                rst_active <= 0;
                if(nack ||stall || timeout ) begin
                   //rsend_cnt <= 7200;
                   rsend_cnt <= 1600;
                   state <= GO_DLY; //Delay X cycles
                    nxt_state <= 80; //Try resending Write STAT
                end 
                if(ack) begin
                   //rsend_cnt <= 300;
                   rsend_cnt <= 360;
                   state <= GO_DLY; 
                   nxt_state <= 100;
                end 
             end





//=================================================================
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
       100: begin
               usb_rdy <=1;
               rst_active  <= 0;
               //rsend_cnt <= 120000;
               rsend_cnt <=   12000;
               state <= GO_DLY; 
               nxt_state <= 101;
            end

       101: begin
               usb_rdy <=1;
               mdata_cap <= 0;
               rst_active <= 0;
               if(device == 0) begin
                  snd <= 1; pid <= READ; addr <= 06; end_pt <= 01;  //LOGITECH M325c & M187
               end else begin
                  snd <= 1; pid <= READ; addr <= 06; end_pt <= 02;  //LOGITECH M325
               end
               state <= WAIT_CMD_DONE; 
               nxt_state <= 102; 
               timeout_cnt <= 1200;
            end

        102: begin
                usb_rdy <=1;
                rst_active <= 0;
                if(nack || stall || timeout) begin
                   rsend_cnt <= 48000; //was 12000
                   state <= GO_DLY; //Delay X cycles
                   nxt_state <= 101; //Try sending Setup to get Descriptor again
                end 
                if(d_v) begin  //NEED to CHECK Data value
                   mdata_cap <= 1;
                   rsend_cnt <= 10;
                   state <= GO_DLY; 
                   nxt_state <= 103;
                   if(device==0) begin  //M325c & M187
                      x_move <= data_out[31:16];  //pos= >   neg= <
                      y_move <= data_out[47:32];  //pos= V   neg= ^
                      btns   <= {5'h00, data_out[2:0]};
                   end else begin
                      x_move <= { {4{data_out[35]}}, data_out[35:24]};  //pos= >   neg= <
                      y_move <= { {4{data_out[47]}}, data_out[47:36]};  //pos= V   neg= ^
                      btns   <= {5'h00, data_out[10:8]};
                   end  
                   scroll <= data_out[55:48];
                end
             end

       103: begin 
               usb_rdy <=1;
               mdata_cap <= 1;
               rst_active <= 0;
               snd <= 1; pid <= ACK ; addr <= 0; end_pt <= 0;
               state <= WAIT_CMD_DONE; 
               nxt_state <= 100; 
            end
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//===============================================================


//----------------------------------------------------------------------
       //Delay for a few cycles
       GO_DLY: begin
              if(rsend_cnt != 0) begin
                 rsend_cnt <= rsend_cnt -1;
              end else begin
                 state <= nxt_state;
              end
           end

       RESTART: begin  //Delay and wait for start again  ///Just for debugging
              //rsend_cnt <= 6000;
              rsend_cnt <= 7200;
              state <= GO_DLY; 
              nxt_state <= 0;
           end

       //Wait for command to finish
       WAIT_CMD_DONE: begin
                         if(host_done==1 && host_done_dly==0)  begin
                            state <= nxt_state; 
                         end
                      end

        //DEFAULT STATE:   go back to the IDLE state
        default: state <=0;
      endcase
   end
end

//assign dp = (dout_en==1) ? dp_o : 1'bz;
//assign dm = (dout_en==1) ? dm_o : 1'bz;


drv_usb drv_usb_inst(.clk60   (clk60),
                     .dp_o    (dp_o),
                     .dm_o    (dm_o),
                     .dout_en (dout_en),
                     .idle    (idle),
                     .rst_n   (chip_rst_n),
                     .full_speed (full_speed),
                     .snd     (snd),
                     .cmd_done (cmd_done),
                     .dev_rst (dev_rst),
                     .stat    (stat),
                     .pid     (pid),
                     .addr    (addr),
                     .end_pt  (end_pt),
                     .data    (snd_data)
                   );


//Only look at what is being received
//assign rcv_dp = (dout_en==0) ? dp : 1'b1;
//assign rcv_dm = (dout_en==0) ? dm : 1'b1;
assign rcv_dp = dp_i;
assign rcv_dm = dm_i;
assign d__oen = dout_en;


wire s_time;
wire pid0,pid1,pid2,pid3;

rcv_usb rcv_usb_inst(.clk60    (clk60),
                     .rst_n    (chip_rst_n),
                     .dp       (rcv_dp),
                     .dm       (rcv_dm),
                     .full_speed (full_speed),
                     .data_out (data_out),
                     .d_v      (d_v),
                     .nack     (nack),
                     .ack      (ack),
                     .stall    (stall),
                     .err      (err),
                     .usb_rst  (usb_rst),
                     .s_time   (s_time),
                     .pid0     (pid0),
                     .pid1     (pid1),
                     .pid2     (pid2),
                     .pid3     (pid3)
                    );

endmodule
