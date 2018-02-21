// DRV_USB
// Creator: Ted Holler
// Date: 1/2/2018
// Gets commands from Host Module and send the commands
// to the USB FULL_SPEED device 
//

module drv_usb(clk60,
                dp_o,
                dm_o,
                dout_en,
                idle,
                rst_n,
                full_speed,
                snd,
                cmd_done,
                dev_rst,
                stat,
                pid,
                addr,
                end_pt,
                data
             );

// 25Mhz/16 = 1.56Mhz  (Low speed USB is 1.5Mhz,  Full speed is 12Mhz)
//parameter MAX_CYCLES=16;
// 50Mhz
//parameter MAX_CYCLES=32;
// 60Mhz/1.5 = 
//parameter MAX_CYCLES=39;  //60mhz Low Speed
parameter MAX_CYCLES=4; //(0-4,   5 cycles)

input       clk60;
input       rst_n;
output      dp_o;
output      dm_o;
output      dout_en;
output      idle;
output      cmd_done;

input       full_speed;
input       snd;
input       dev_rst;
input       stat;
input[3:0]  pid;
input[6:0]  addr;
input[3:0]  end_pt;
input[63:0] data;

wire[3:0] pid;

reg[5:0]  clk_cycle;
reg[15:0] bit_cnt;
reg       dp_o;
reg       dm_o;
reg       dout_en;
reg       idle;
reg       do_sof;

reg       snd_pending;
reg       snd_clr;
reg[3:0]  sync_cnt;
reg[7:0]  cnt;
reg[5:0]  ones_cnt;
reg[15:0] sof_cnt;

reg       cmd_done;
reg       snd_r;
reg       dev_rst_r;
reg       stat_r;
reg[7:0]  sync_r;
reg[7:0]  sof_r;
reg[7:0]  pid_r;
reg[15:0] token_r;
reg[3:0]  end_pt_r;
reg[79:0] data_r;

reg[20:0]  dev_rst_cnt;
reg[14:0]  keep_alive_cnt;
reg[3:0]   state;

reg crc4;
reg crc3;
reg crc2;
reg crc1;
reg crc0;
reg[3:0] znt;
reg[5:0] pop_cnt;

parameter [3:0] IDLE     =0,
                SYNC     =1,
                PID      =2,
                ADDR     =3,
                END_PNT  =4,
                TOKEN    =5,
                DATA     =6,
                HAND_SHK =7,
                STOP     =8,
                DEV_RST  =9,
                STAT     =10,
                GO_SOF_D =12;

reg [7:0]  dznt;
reg [15:0] crc16_cap;

wire [15:0] crc16_out;
wire        crc_en;

always @(posedge clk60) begin
   if(~rst_n) begin
      pid_r     <=0;
      token_r   <= 0;
      end_pt_r  <=0;
      data_r    <= 80'h0;
      sync_r    <= 8'h80;
      sof_r     <= 8'hA5;  //pid=4'b0101
      dev_rst_r <= 0;
      stat_r    <= 0;
   end else begin
      snd_r <= snd;
      if(snd==1) begin
         dev_rst_r <= dev_rst;
         pid_r    <= {~pid[3],~pid[2],~pid[1],~pid[0], pid[3],pid[2],pid[1],pid[0]};
         token_r  <= {5'h00 , end_pt, addr};
         data_r[63:0] <= data;
         stat_r       <= stat;
      end
      if(state==DEV_RST) begin
         dev_rst_r <= 0;
      end
      if(state==TOKEN && cnt==2) begin
         token_r[15:11] <= {crc0,crc1,crc2,crc3,crc4};
      end
   end
end

always @(posedge clk60) begin
   if(~rst_n) begin
      clk_cycle <= 0;
   end else begin
      clk_cycle <= (clk_cycle==MAX_CYCLES) ? 0 :clk_cycle+1;
   end
end

always @(posedge clk60) begin
   snd_pending <= (rst_n==0 || (snd_clr==1 && clk_cycle!= MAX_CYCLES) ) ? 0 : snd==1 ? 1 : snd_pending;
end

always @(posedge clk60) begin
   if(~rst_n) begin
      state    <= IDLE;
      snd_clr  <= 0; 
      sync_cnt <= 8;
      dout_en <= 0;
      dp_o <= 1;  //High Speed (Resistor will provide)
      dm_o <= 0;  //LOW speed (Resistor will provide this)
      cnt  <= 0;
      dev_rst_cnt <=0;
      keep_alive_cnt <=0;
      ones_cnt <= 0;
      idle <= 0;
      cmd_done <=0;
      do_sof <=0;
   end else begin
      snd_clr <= 0; 
      cmd_done <=0;
      if(clk_cycle==MAX_CYCLES) begin
         idle <= 0;
         case(state) 
           IDLE: begin
                    do_sof <= 0;
                    idle <= 1'b1;
                    keep_alive_cnt <= (keep_alive_cnt != 15'h2EDF) ? keep_alive_cnt+1 : 15'h2EDF;
                    if((keep_alive_cnt==15'h2EDF) && (snd_pending==0) && (snd_r ==0 )) begin
                       sof_cnt <= sof_cnt+1;

                       do_sof <=1;
                       state <= SYNC;
                       //snd_clr <= 1; 
                       sync_cnt <= 0;
                       dout_en <= 1;
                       //dp_o <= 0; 
                       //dm_o <= 1;
                    end
                    if( (snd_pending==1 || snd_r==1) && dev_rst_r==0 ) begin
                       state <= SYNC;
                       snd_clr <= 1; 
                       sync_cnt <= 0;
                       dout_en <= 1;
                       //dp_o <= 1;
                       //dm_o <= 0;
                    end
                    if( (snd_pending==1 || snd_r==1) && dev_rst_r==1 ) begin
                       state <= DEV_RST;
                       //dev_rst_cnt <= 21'h1_3BF2; //1375000 (55msec)
                       dev_rst_cnt <= 21'h2_77e4; //1375000 (55msec)
                       //dev_rst_cnt <= 21'hFF; //1375000 (55msec)---for sim only
                       snd_clr <= 1; 
                       dout_en <= 1;
                       dp_o <= 0;
                       dm_o <= 0;
                    end
                 end //IDLE
           SYNC: begin    //   01010100   
                    if(sync_r[sync_cnt]==0) begin
                       dout_en <= 1;
                       dp_o <= ~dp_o;
                       dm_o <= ~dm_o;
                    end

                    if(sync_cnt == 7 ) begin
                       cnt <= 0;
                       state<=PID;
                    end 

                    sync_cnt <= sync_cnt+1;
                 end //SYNC
           PID: begin
                   if((do_sof==0 && pid_r[cnt]==0) || (do_sof==1 && sof_r[cnt]==0) ) begin
                      dout_en <= 1;
                      dp_o <= ~dp_o;
                      dm_o <= ~dm_o;
                   end

                   if(cnt !=7) begin
                      cnt <= cnt+1; 
                   end else begin
                      cnt <= 0;
                      case({do_sof,pid_r[1:0]}) 
                        3'b000:  begin //SPECIAL
                                   state <= STOP;
                                end
                        3'b001:  begin //TOKEN
                                   state <= TOKEN;
                                end
                        3'b010:  begin //HAND_SHK
                                   state <= STOP;
                                end
                        3'b011:  begin //DATA0 or DATA1
                                   if(stat_r ==0) begin
                                      state <= DATA;
                                   end else begin
                                      state <= STAT;
                                   end
                                end
                        3'b100:  begin //SOF Data
                                     state <= GO_SOF_D;
                                 end
                        3'b101:  begin //SOF Data
                                     state <= GO_SOF_D;
                                 end
                        3'b110:  begin //SOF Data
                                     state <= GO_SOF_D;
                                 end
                        3'b111:  begin //SOF Data
                                     state <= GO_SOF_D;
                                 end
                      endcase
                   end
                end //PID
            TOKEN: begin
                      ones_cnt <= 0;
                      if(token_r[cnt]==0) begin
                         dout_en <= 1;
                         dp_o <= ~dp_o;
                         dm_o <= ~dm_o;
                      end
                      if(cnt !=  15) begin
                         cnt <= cnt+1;
                      end else begin
                         do_sof <= 0;
                         cnt  <= 0;
                         state <= STOP;
                      end
                   end//TOKEN
            DATA: begin
                     //NOTE:   FIXME  There is potential that DATA_PID CMD and the first few bits will result in 6x1's
                     ones_cnt[5:0] <= (cnt<64) ? {ones_cnt[4:0], data_r[cnt]} : {ones_cnt[4:0], crc16_cap[cnt-64]};

                     if((data_r[cnt]==0 && cnt<64) || (crc16_cap[cnt-64]==0 && cnt>=64) || (ones_cnt==6'h3f)) begin
                        dout_en <= 1;
                        dp_o <= ~dp_o;
                        dm_o <= ~dm_o;
                        ones_cnt <= 0;
                     end else begin
                       //NOTE:   FIXME  There is potential that DATA_PID CMD and the first few bits will result in 6x1's
                       ones_cnt[5:0] <= (cnt<64) ? {ones_cnt[4:0], data_r[cnt]} : {ones_cnt[4:0], crc16_cap[cnt-64]};
                     end

                     if(cnt !=  79 )begin
                        if(ones_cnt!=6'h3f ) begin
                           cnt <= cnt+1;
                        end
                     end else begin
                        cnt  <=0;
                        state <= STOP;
                     end
                  end//DATA
            HAND_SHK: begin
                         //if(hand[cnt]==0) begin
                         //   dp_o <= ~dp_o;
                         //   dm_o <= ~dm_o;
                         //end
                         //if(cnt !=  79) begin
                         //   cnt <= cnt+1;
                         //end else begin
                         //   state <= STOP;
                         //   cnt <= 0;
                         //end
                      end
             STOP: begin
                        cnt <= cnt+1;
                        case(cnt) 
                           0: begin
                                 dout_en <= 1;
                                 dp_o <= 0;
                                 dm_o <= 0;
                              end
                           1: begin
                                 keep_alive_cnt <=0;
                                 dout_en <= 1;
                                 dp_o <= 1;  //Full SPEED   (Resistor will determine this)
                                 dm_o <= 0;  //1=LOW SPEED
                              end
                           2: begin
                                 keep_alive_cnt <=0;
                                 dout_en <= 1;
                                 dp_o <= 1;  //Full SPEED   (Resistor will determine this)
                                 dm_o <= 0;  //1=LOW SPEED
                              end
                           3: begin
                                 keep_alive_cnt <=0;
                                 dout_en <= 1;
                                 dp_o <= 1;  //Full SPEED   (Resistor will determine this)
                                 dm_o <= 0;  //1=LOW SPEED
                              end
                           4: begin
                                 keep_alive_cnt <=0;
                                 dout_en <= 0;
                                 dp_o <= 1;  //Full SPEED   (Resistor will determine this)
                                 dm_o <= 0;  //1=LOW SPEED
                              end
                           5: begin
                                 cmd_done <=1;
                                 cnt <= 0;
                                 state <= IDLE;
                              end
                        endcase
                   end

             DEV_RST: begin
                         dev_rst_cnt <= (dev_rst_cnt!=0) ? dev_rst_cnt-1: 0;
                         dout_en  <= 1;
                         dp_o     <= 0;  
                         dm_o     <= 0;
                         cmd_done <= 0;
                         if(dev_rst_cnt==1) begin
                            cnt      <= 0;
                            cmd_done <=1;
                            state    <= IDLE;
                         end
                         if(dev_rst_cnt<3) begin
                            keep_alive_cnt <=0;
                            cnt     <= 0;
                            dout_en <= 0;
                            dp_o    <= 1; //1=FULL SPEED
                            dm_o    <= 0; //1=Low SPEED
                         end
                      end 
            STAT: begin
                     dout_en <= 1;
                     dp_o <= ~dp_o;
                     dm_o <= ~dm_o;
                     if(cnt !=  15) begin
                        cnt <= cnt+1;
                     end else begin
                        cnt  <=0;
                        state <= STOP;
                     end
                  end
            GO_SOF_D: begin
                         do_sof <= 0;
                         ones_cnt[5:0] <= {ones_cnt[4:0], sof_cnt[cnt]};
                         if(cnt==3) sof_cnt[15:11] <= {crc0,crc1,crc2,crc3,crc4};

                         if( (sof_cnt[cnt]==0 )  || (ones_cnt==6'h3f)) begin
                            dout_en <= 1;
                            dp_o <= ~dp_o;
                            dm_o <= ~dm_o;
                            ones_cnt <= 0;
                         end else begin
                           //NOTE:   FIXME  There is potential that DATA_PID CMD and the first few bits will result in 6x1's
                           ones_cnt[5:0] <= {ones_cnt[4:0], sof_cnt[cnt]} ;
                         end

                         if(cnt < 16   )begin
                            if(ones_cnt!=6'h3f ) begin
                               cnt <= cnt+1;
                            end
                         end else begin
                            dp_o <= 0;
                            dm_o <= 0;
                            cnt  <=0;
                            state <= STOP;
                         end
                      end
         endcase
      end //if(clk_cycle == MAX_CYCLES=16) 
   end
end

//wire crc_in = (state==TOKEN ) ? token_r[znt] : (state==GO_SOF_D ) ? sof_cnt[znt] : 0;
//wire crc_in = (do_sof==0 ) ? token_r[znt] : sof_cnt[znt] ;
wire crc_in = (state!= GO_SOF_D ) ? token_r[znt] : sof_cnt[znt] ;


always @(posedge clk60) begin
   if( state==TOKEN || state==GO_SOF_D ) begin
      pop_cnt <= {pop_cnt[4:0], crc_in};
      if( znt != 11) begin
         crc4 <= crc3;
         crc3 <= crc2;
         //crc2 <= ~(crc1 ^ crc4 ^ token_r[znt]);
         crc2 <= ~(crc1 ^ crc4 ^ crc_in);
         crc1 <= crc0;
         //crc0 <= crc4 ^ token_r[znt];
         crc0 <= crc4 ^ crc_in;
       end
      //znt <= pop_cnt=='h3f ? znt : (znt!=11) ? znt+1 : 11;
      znt <= (znt!=11) ? znt+1 : 11;
   end else begin
      pop_cnt <= 6'h01;  //Token and SOF pid[7]==1
      znt <= 0;
      crc4 <= 0;
      crc3 <= 0;
      crc2 <= 0;
      crc1 <= 0;
      crc0 <= 0;
   end
end


always @(posedge clk60) begin
   case(state) 
       DATA: begin
                dznt <= (dznt!=64) ? dznt+1 : 64;
                if(dznt==64) crc16_cap <= crc16_out;
             end
      GO_SOF_D: begin
                   dznt <= (dznt!=11) ? dznt+1 : 11;
                   if(dznt==11) crc16_cap <= crc16_out;
                end
      default: begin
                  dznt <= 0;
                  crc16_cap <=0;
               end
   endcase
end

assign crc_en = ((dznt>=0 && dznt<=63) && state==DATA) ? 1'b1 : 
                ((dznt>=0 && dznt<=10) && state==GO_SOF_D ) ? 1'b1 : 1'b0 ;


//wire crc_in = (state==DATA ) ? data_r[dznt] : (state==GO_SOF_D ) ? sof_cnt[dznt] : 0;


crc crc_inst(.clk (clk60),
          .crc_en (crc_en),
          .crc_out (crc16_out),
          //.rst     ((state==PID && cnt==2) || (state==GO_SOF_P && cnt==2)),
          .rst     (state==PID && cnt==2),
          .data_in (data_r[dznt])
         );

endmodule
