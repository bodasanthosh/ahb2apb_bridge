module ahb_master(Hclk, Hresetn, Hreadyout, Hrdata, Haddr, Hwdata, Hwrite, Hreadyin, Htrans);

input Hclk, Hresetn, Hreadyout;
input [31:0] Hrdata;
output reg [31:0] Haddr, Hwdata;
output reg Hwrite, Hreadyin;
output reg [1:0] Htrans;

reg [2:0] Hburst, Hsize;

task single_write;
begin
  @(posedge Hclk)
  #1;
  begin
      Hwrite = 1;
      Htrans = 2'd2;
      Hsize = 0;
      Hburst = 0;
      Hreadyin = 1;
      Haddr = 32'h8000_0001;
  end
  @(posedge Hclk)
  #1;
  begin
      Htrans = 2'd0;
      Hwdata = 8'h80;
  end
end
endtask

task single_read;
begin
  @(posedge Hclk)
  #1;
  begin
      Hwrite = 0;
      Htrans = 2'd2;
      Hsize = 0;
      Hburst = 0;
      Hreadyin = 1;
      Haddr = 32'h8000_0001;
  end
  @(posedge Hclk)
  #1;
  begin
      Htrans = 2'd0;
  end
end
endtask


endmodule


module ahb_slave_interface(Hclk, Hresetn, Hwrite, Hreadyin, Htrans, Haddr, Hwdata, Hresp, Hrdata, valid, Haddr1, Haddr2, Hwdata1, Hwdata2, Hwritereg1, Hwritereg2, temp_selx, prdata);
input Hclk, Hresetn, Hreadyin, Hwrite, prdata;
input [1:0] Htrans;
input [31:0] Haddr, Hwdata;
output [1:0] Hresp;
output [31:0] Hrdata;
output reg Hwritereg1;
output reg Hwritereg2;
output reg valid;
output reg [31:0] Haddr1, Haddr2, Hwdata1, Hwdata2;
output reg [2:0] temp_selx;


always @(posedge Hclk) begin
    if (!Hresetn)
    begin
        Haddr1 <= 0;
        Haddr2 <= 0;
    end
    else begin
        Haddr1 <= Haddr;
        Haddr2 <= Haddr1;
    end
end

always @(posedge Hclk) begin
    if (!Hresetn)
    begin
        Hwdata1 <= 0;
        Hwdata2 <= 0;
    end
    else begin
        Hwdata1 <= Hwdata;
        Hwdata2 <= Hwdata1;
    end
    
end

always @(posedge Hclk) begin
    if(!Hresetn) begin
        Hwritereg1 <= 0;
        Hwritereg2 <= 0;
    end
    else begin
        Hwritereg1 <= Hwrite;
        Hwritereg2 <= Hwritereg1;    
    end
    
end

always @(*) begin
    valid = 1'b0;
    if(Hreadyin && Haddr >= 32'h8000_000 && Haddr < 32'h8C00_0000 && Htrans == 2'b10 || Htrans == 2'b11) begin
        valid = 1 ;
    end
    else begin
        valid = 0;
    end
end

always @(*) begin
    temp_selx = 3'b000;
    if(Haddr >= 32'h8000_0000 && Haddr < 32'h8400_0000) begin
        temp_selx = 3'b001;
    end
    else if(Haddr >= 32'h8400_0000 && Haddr < 32'h8800_0000) begin
        temp_selx = 3'b010;
    end
    else if(Haddr >= 32'h8800_0000 && Haddr < 32'h8C00_0000) begin
        temp_selx = 3'b011;
    end
end
assign Hrdata = prdata;

endmodule



module apb_controller(Hclk, Hresetn, Hwrite, Hwritereg, valid, Haddr, Haddr1, Haddr2, Hwdata, Hwdata1, Hwdata2, prdata, temp_sel, penable, pwrite, HReadyout, psel, paddr, pwdata);

input Hclk, Hresetn, Hwrite, Hwritereg, valid;
input [31:0] Haddr, Haddr1, Haddr2, Hwdata, Hwdata1, Hwdata2, prdata;
output reg penable, pwrite;
output reg HReadyout;
output reg [2:0]psel;
output reg [2:0] temp_sel;
output reg [31:0] paddr, pwdata;

reg HReadyout_temp, penable_temp, pwrite_temp;
reg [31:0]pwdata_temp, paddr_temp;
reg [2:0]psel_temp;

parameter ST_IDLE = 3'b000,
          ST_READ = 3'b001,
          ST_RENABLE = 3'b010,
          ST_WWAIT = 3'b011,
          ST_WRITE = 3'b100,
          ST_WRITEP = 3'b101,
          ST_WENABLEP = 3'b110,
          ST_WENABLE = 3'b111;

reg [2:0] present_state, next_state;

always @(posedge Hclk, negedge Hresetn) begin
    if(!Hresetn) begin
        present_state <= ST_IDLE;
    end    
    else begin
        present_state <= next_state;
    end
end

always @(*) begin
    next_state = ST_IDLE;
    case(present_state)
    ST_IDLE : begin
                if (valid && !Hwrite)
                    next_state = ST_READ;
                else if (valid && Hwrite)
                    next_state = ST_WWAIT;
                else
                    next_state = ST_IDLE;        
              end

    ST_READ : next_state = ST_RENABLE;
              
    ST_RENABLE : begin
                    if (valid && !Hwrite)
                        next_state = ST_READ;
                    else if (valid && Hwrite)
                        next_state = ST_WWAIT;
                    else
                        next_state = ST_IDLE;        
                 end
    
    ST_WWAIT : begin
                if (valid)
                    next_state = ST_WRITEP;
                else
                    next_state = ST_WRITE;        
              end
    
    ST_WRITE : begin
                 if (valid)
                    next_state = ST_WENABLEP;
                 else
                    next_state = ST_WENABLE;        
               end
    
    ST_WRITEP : next_state = ST_WENABLEP;
                
    ST_WENABLEP : begin
                    if (valid && Hwritereg)
                        next_state = ST_WRITEP;
                    else if (!valid && Hwritereg)
                        next_state = ST_WRITE;
                    else
                        next_state = ST_READ;        
                  end

    ST_WENABLE : begin
                    if (valid && !Hwrite)
                        next_state = ST_READ;
                    else if (valid && Hwrite)
                        next_state = ST_WWAIT;
                    else
                        next_state = ST_IDLE;        
                 end
    endcase
end


always @(*) begin
    case(present_state)
    ST_IDLE : begin
                if (valid && !Hwrite) begin
                    pwrite_temp = Hwrite;
                    HReadyout_temp = 0;
                    psel_temp = temp_sel;
                    penable_temp = 0;
                    paddr_temp = Haddr;
                end
                
                else if (valid && Hwrite)begin
                    psel_temp = 0;
                    penable_temp = 0;
                    HReadyout_temp = 1;
                end

                else begin
                    HReadyout_temp = 1;
                    psel_temp = 0;
                    penable_temp = 0;
                end        
              end

    ST_READ : begin
                penable_temp = 1;
                HReadyout_temp = 1;        
              end
              
    ST_RENABLE : begin
                if (valid && !Hwrite) begin
                    pwrite_temp = Hwrite;
                    HReadyout_temp = 0;
                    psel_temp = temp_sel;
                    penable_temp = 0;
                    paddr_temp = Haddr;
                end
                
                else if (valid && Hwrite)begin
                    psel_temp = 0;
                    penable_temp = 0;
                    HReadyout_temp = 1;
                end

                else begin
                    HReadyout_temp = 1;
                    psel_temp = 0;
                    penable_temp = 0;
                end        
              end
    
    ST_WWAIT : begin
                paddr_temp = Haddr1;
                pwdata_temp = Hwdata;
                pwrite_temp = Hwrite;
                psel_temp = temp_sel;
                penable_temp = 0;
                HReadyout_temp = 0;        
              end
    
    ST_WRITE : begin
                 penable_temp = 1;
                 HReadyout_temp =1;        
               end
    
    ST_WRITEP : begin
                    penable_temp = 1;
                    HReadyout_temp =1;
                end
                
    ST_WENABLEP : begin
                    if ((!valid && Hwritereg )|| (valid && Hwritereg)) begin
                        paddr_temp = Haddr1;
                        pwdata_temp = Hwdata;
                        pwrite_temp = Hwrite;
                        psel_temp = temp_sel;
                        penable_temp = 0;
                        HReadyout_temp = 0;        
                    end

                    else if(!Hwritereg) begin
                        pwrite_temp = Hwrite;
                        HReadyout_temp = 0;
                        psel_temp = temp_sel;
                        penable_temp = 0;
                        paddr_temp = Haddr;
                    end
                 end

    ST_WENABLE : begin
                    if(valid && Hwrite) begin
                        psel_temp = 0;
                        penable_temp = 0;
                        HReadyout_temp = 1;  
                    end
                    else if(valid && !Hwrite) begin
                        pwrite_temp = Hwrite;
                        HReadyout_temp = 0;
                        psel_temp = temp_sel;
                        penable_temp = 0;
                        paddr_temp = Haddr;                                            
                    end
                    else begin
                        HReadyout_temp = 1;
                        psel_temp = 0;
                        penable_temp = 0;
                    end
                            
                 end
    endcase
end

//Output Logic

always @(posedge Hclk ) begin
    if(!Hresetn) begin
        paddr <= 0;
        pwdata <= 0;
        pwrite <= 0;
        psel <= 0;
        penable <= 0;
        HReadyout <= 1;
    end
    else begin
        paddr <= paddr_temp;
        pwdata <= pwdata_temp;
        pwrite <= pwrite_temp;
        psel <= psel_temp;
        penable <= penable_temp;
        HReadyout <= HReadyout_temp;
    end
end
module APB_Interface( 
 input pwrite,penable, 
 input [2:0] pselx, 
 input [31:0] paddr,pwdata, 
 output pwrite_out,penable_out, 
 output [2:0] psel_out, 
 output [31:0] paddr_out,pwdata_out, 
 output reg [31:0] prdata);

assign pwrite_out = pwrite;
assign paddr_out = paddr;
assign psel_out = pselx;
assign pwdata_out = pwdata;
assign penable_out = penable;

always @*
	begin
		if(!pwrite && penable)
				begin
					prdata = 8'd25;
				end
	end
	
endmodule

module Bridge_Top(
 input Hclk, Hresetn, Hwrite, Hreadyin,
 input [1:0] Htrans,
 input [31:0] Hwdata, Haddr, prdata,
 output penable, pwrite,HReadyout,
 output [2:0] psel,
 output [1:0] Hresp,
 output [31:0] paddr, pwdata, Hrdata);
wire [31:0] Hwdata1, Hwdata2, Haddr1, Haddr2;
wire [2:0] temp_selx;
wire Hwritereg, Hwritereg1,Hwritereg2;
wire valid;

AHB_Slave AHB_SLAVE(Hclk, Hresetn, Hwrite, Hreadyin, Htrans, Haddr, Hwdata, Hresp,Hrdata, valid, Haddr1, Haddr2, Hwdata1, Hwdata2, Hwritereg1, Hwritereg2, temp_selx, prdata);
APB_Controller APB_CONTROLLER(Hclk, Hresetn, Hwrite, Hwritereg, valid, Haddr, Haddr1, Haddr2,Hwdata, Hwdata1, Hwdata2, prdata, temp_selx, penable, pwrite, HReadyout, psel, paddr, pwdata);
