// RCV_USB
// Creator: Ted Holler
// Date: 1/2/2018
// Capture commands from the USB FULL_SPEED device
// and send the information to the Host Module


module rcv_usb(clk60,  //60Mhz
               rst_n,
               dp,
               dm,
               full_speed,
               data_out,
               d_v,
               nack,
               ack,
               stall,
               err,
               usb_rst,
               s_time,
               pid0,
               pid1,
               pid2,
               pid3
              );

// 25Mhz/16 = 1.56Mhz  (Low speed USB is 1.5Mhz,  Full speed is 12Mhz)
//parameter MAX_CYCLES=16;
//parameter MAX_CYCLES=32;
//parameter MAX_CYCLES=41; //60Mhz Low speed
//parameter MAX_CYCLES=5;
parameter MAX_CYCLES=4;

input         clk60;  //60Mhz
input         rst_n;
input         dp;
input         dm;
input         full_speed;
output[63:0]  data_out  /* synthesis keep */;
output        d_v;
output        nack;
output        stall;
output        ack;
output        err;
output        usb_rst;
output        s_time;
output        pid0;
output        pid1;
output        pid2;
output        pid3;


reg[63:0] data_out;
reg       d_v;
reg       nack;
reg       ack;
reg       stall;
reg       err;
reg       usb_rst;



reg[3:0]  pid /* synthesis preserve */;
reg[79:0] cap_data  /* synthesis preserve */ ;
reg[15:0] crc_data ;
reg[6:0]  cap_cnt;
reg[6:0]  cap_crc_cnt;
reg[5:0]  ones_cnt;
reg[6:0]  data_cnt /* synthesis preserve */;

reg       xfer_done;
reg       data_start;
wire      cap_active;
reg[6:0]  addr;
reg[3:0]  end_pt;

reg[7:0] sync_bits;
reg[5:0] cycle_time;
wire     new_xfer;

reg       sample0;
reg       sample1;
reg       usb_data;
reg[15:0] bit_cnt;
//reg[15:0] bbit_cnt;

always @(posedge clk60) begin
  if(!rst_n) begin
     data_out <= 64'h0;
     d_v      <= 0;
     nack     <= 0; 
     stall    <= 0;
     ack      <= 0; 
     err      <= 0;
  end else begin
     if(xfer_done==1) begin
        //data_out <= cap_data[79:16]; //May need to have bytes valid..... ????
        data_out <= (data_cnt=='h56) ? cap_data[63:0] : {32'h0, cap_data[63:32]};

        d_v      <= (pid == 3 || pid==11);  //Data0 or Data1 is valid
        ack      <=  pid == 2;                //Ack
        nack     <=  pid == 10;               //NAck
        stall    <=  pid == 14;               //STall
        err      <=  0;
     end else begin
        d_v  <=  0;
        ack  <=  0;
        nack <=  0;
        stall <= 0;
        err  <=  0;
     end
  end
end


reg  dp_d,dp_d1,dp_d2,dp_d3;
reg  dm_d,dm_d1,dm_d2,dm_d3;

reg  pre_xfer_active;
reg  pre_xfer_active1;
reg  xfer_active;
reg  xxfer_active;
reg  xfer_active_dly;
reg xfer_start;
reg [21:0] usb_rst_cnt;


always @(posedge clk60) begin
   if(!rst_n) begin
      usb_rst <= 0;
      usb_rst_cnt <= 0;
   end else begin
      
      //*********************************************************************
      usb_rst <= (usb_rst_cnt >= 22'h325AA0) ? 1'b1 : 1'b0;  //60Mhz

      if( dp_d2 == 0 && dm_d2==0) begin 
         usb_rst_cnt <= (usb_rst_cnt != 22'h325AA0) ? usb_rst_cnt+1 : 22'h325AA0;
      end else begin
         usb_rst_cnt <= 0;
      end
   end  
end

always @(posedge clk60) begin
   xfer_start  <=  dp_d2==0 && dm_d2==1 && (dp_d3 != dp_d2 )&& xfer_active==0 && rst_n==1;
end




assign s_time = cycle_time==1;
assign pid0 = ^sync_bits;
assign pid1 = 0; //^bbit_cnt;
assign pid2 = xxfer_active;
assign pid3 = cycle_time[5];




always @(posedge clk60) begin
  dm_d3 <= dm_d2;
  dm_d2 <= dm_d1;
  dm_d1 <= dm_d;
  dm_d  <= dm;

  dp_d3 <= dp_d2;
  dp_d2 <= dp_d1;
  dp_d1 <= dp_d;
  dp_d  <= dp;
end


always @(posedge clk60) begin
  xfer_active_dly <=  xfer_active;
  
  if(!rst_n) begin
     pre_xfer_active <= 0;
     xfer_active_dly <=0;
     pre_xfer_active <= 0;
     pre_xfer_active1 <= 0;
  end else begin
     //             Start                           // EOP                             
     xfer_active <= xfer_start            ? 1'b1 : ((dp_d2==0 && dm_d2==0) && (cycle_time==1) )  ? 0 : xfer_active;
     xxfer_active <= (sync_bits[4:0]==1 ) ? 1'b1 : ((dp_d2==0 && dm_d2==0) && (cycle_time==1) )  ? 0 : xxfer_active;
  end
end

assign   new_xfer = (dp_d2 != dp_d3);

always @(posedge clk60) begin
   if(!rst_n) begin
      cycle_time <= 0 ;
      sync_bits <= 255;
   end else begin
      cycle_time <= (xfer_start || (xfer_active==1 && cycle_time>=MAX_CYCLES) || new_xfer ) ? 0 : (xfer_active==0) ? MAX_CYCLES : cycle_time+1;
      //sync_bits <= (bbit_cnt>10) ? 255 : (s_time==1) ? {sync_bits[6:0], (dp_d2== sample0 && dm_d2==sample1 && dp_d2 != dm_d2)  } :  sync_bits;
      sync_bits <= (bit_cnt>10) ? 255 : (s_time==1) ? {sync_bits[6:0], (dp_d2== sample0 && dm_d2==sample1 && dp_d2 != dm_d2)  } :  sync_bits;
   end
end


//Sample the data in the middle (??? or would be be better on the left side) of the EYE.
//
always @(posedge clk60) begin
  //                                vvvvv  1+ SAMPLE_TIME
  //bit_cnt <=   xfer_active==0 ? 0 : ( cycle_time==2 ) ? bit_cnt+1 : bit_cnt;
  //bbit_cnt <= xxfer_active==0 ? 8 : ( cycle_time==2 ) ? bbit_cnt+1 : bbit_cnt;

  bit_cnt <=   xxfer_active==0 ? 8 : ( cycle_time==2 ) ? bit_cnt+1 : bit_cnt;

  if( cycle_time==1 ) begin
     sample0 <= dp_d2;
     sample1 <= dm_d2;
     usb_data <= (dp_d2== sample0 && dm_d2==sample1 && dp_d2 != dm_d2) ? 1 : 0;
  end 
end

wire   sync;
wire   pid0;
wire   pid1;
wire   pid2;
wire   pid3;



always @(posedge clk60) begin
                          //if(xfer_active==0) begin
                          //if(!rst_n) begin
  if(cycle_time==1) begin
     //if(bit_cnt==1) begin
     if(bit_cnt<9) begin
        pid    <=0;
        addr   <=0;
        end_pt <=0;
     end else begin
        if(bit_cnt==9  ) pid[0] <= usb_data;
        if(bit_cnt==10 ) pid[1] <= usb_data;
        if(bit_cnt==11 ) pid[2] <= usb_data;
        if(bit_cnt==12 ) pid[3] <= usb_data;
                            

        if(bit_cnt==17 && pid[1:0]==2'b01) addr[0] <= usb_data;
        if(bit_cnt==18 && pid[1:0]==2'b01) addr[1] <= usb_data;
        if(bit_cnt==19 && pid[1:0]==2'b01) addr[2] <= usb_data;
        if(bit_cnt==20 && pid[1:0]==2'b01) addr[3] <= usb_data;
        if(bit_cnt==21 && pid[1:0]==2'b01) addr[4] <= usb_data;
        if(bit_cnt==22 && pid[1:0]==2'b01) addr[5] <= usb_data;
        if(bit_cnt==23 && pid[1:0]==2'b01) addr[6] <= usb_data;

        if(bit_cnt==24 && pid[1:0]==2'b01) end_pt[0] <= usb_data;
        if(bit_cnt==25 && pid[1:0]==2'b01) end_pt[1] <= usb_data;
        if(bit_cnt==26 && pid[1:0]==2'b01) end_pt[2] <= usb_data;
        if(bit_cnt==27 && pid[1:0]==2'b01) end_pt[3] <= usb_data;
     end
  end
end

assign cap_active =  xfer_active==1 && ( pid==3 || pid== 11) &&  (cycle_time==1 ) &&
                     (bit_cnt>='h11 && bit_cnt<='h61 );

always @(posedge clk60) begin
  //if(!rst_n) begin
  //if(!rst_n || bit_cnt==1) begin
  if(!rst_n || bit_cnt<9) begin
     xfer_done <= 0;
     data_start <= 0;
     cap_data[79:0] <= 80'hffffffffffffffffffff;
     crc_data <= 16'h0000;
     ones_cnt <= 6'h00; 
     data_cnt <= 7'h00;
  end else begin
     //                       DATA0 ||  DATA1
     if( xfer_active==1 && ( pid==3 || pid== 11) &&  (cycle_time==1) ) begin

        //TODO:   Add code to Count # of 1 bits and skip the "0_ADDED' bit 
        //cap_data[79:0] <= (bit_cnt>='h11 && bit_cnt<='h50 ) ? {usb_data, cap_data[79:1] } : cap_data[79:0];


        ones_cnt[5:0]  <= { ones_cnt[4:0], usb_data };
        cap_data[79:0] <= ( bit_cnt>='h11 && ones_cnt!=6'h3f ) ? {usb_data, cap_data[79:1] } : cap_data[79:0];
        if(ones_cnt!=6'h3f) data_cnt <= data_cnt+1;

        crc_data[15:0] <= (bit_cnt>='h51 && bit_cnt<='h60 ) ? {usb_data, crc_data[15:1] } : crc_data[15:0];
     end

     xfer_done    <=  (xfer_active==0 && xfer_active_dly==1 )  ?  1'b1 : 1'b0;
     data_start <= (( xfer_active==1 && ( pid==3 || pid== 11) &&  cycle_time==1 ) &&  bit_cnt== 14 ) ?  1'b1 : 1'b0;
  end
end



endmodule
