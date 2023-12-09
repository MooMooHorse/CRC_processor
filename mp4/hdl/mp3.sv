module mp3
import rv32i_types::*;
(
    input   logic           clk,
    input   logic           rst,
    output  logic   [31:0]  bmem_address,
    output  logic           bmem_read,
    output  logic           bmem_write,
    input   logic   [63:0]  bmem_rdata,
    output  logic   [63:0]  bmem_wdata,
    input   logic           bmem_resp
);
    /* WIRES FROM CPU TO BUS ADAPTOR */
    logic mem_read; // cpu -> cache
    logic mem_write; // cpu -> cache
    rv32i_word mem_address; // cpu -> bus adaptor
    rv32i_word mem_rdata_cpu; // bus adaptor -> cpu
    logic [3:0] mem_byte_enable_cpu; // cpu -> bus adaptor
    rv32i_word mem_wdata_cpu; // cpu -> bus adaptor 
    logic mem_resp; // bus adaptor -> cpu

    /* WIRES FROM BUS ADAPTOR TO CACHE */
    logic [255:0] mem_wdata256;
    logic [255:0] mem_rdata256;
    logic [31:0] mem_byte_enable256;

    /* WIRES FROM CACHE TO CACHLINE ADAPTOR */
    logic [255:0] pmem_rdata;   
    logic [255:0] pmem_wdata;
    logic [31:0] pmem_address;
    logic pmem_resp, pmem_read, pmem_write;
    
    cpu cpu(.clk(clk), .rst(rst), .mem_resp(mem_resp), .mem_rdata(mem_rdata_cpu), 
    .mem_read(mem_read), .mem_write(mem_write), .mem_byte_enable(mem_byte_enable_cpu), 
    .mem_address(mem_address), .mem_wdata(mem_wdata_cpu));
    

    bus_adapter bus_adapter(.address(mem_address), .mem_rdata256(mem_rdata256), .mem_rdata(mem_rdata_cpu),
    .mem_wdata256(mem_wdata256), .mem_wdata(mem_wdata_cpu), .mem_byte_enable256(mem_byte_enable256), 
    .mem_byte_enable(mem_byte_enable_cpu));

    cache cache(.clk(clk), .rst(rst), .mem_address(mem_address), .mem_read(mem_read), .mem_write(mem_write),
    .mem_byte_enable(mem_byte_enable256), .mem_wdata(mem_wdata256), .mem_rdata(mem_rdata256), .mem_resp(mem_resp),
    .pmem_address(pmem_address), .pmem_read(pmem_read), .pmem_write(pmem_write),
    .pmem_rdata(pmem_rdata), .pmem_wdata(pmem_wdata), .pmem_resp(pmem_resp));

    cacheline_adaptor cacheline_adaptor(.clk(clk), .reset_n(~rst), .line_i(pmem_wdata), .line_o(pmem_rdata), .address_i(pmem_address), 
    .read_i(pmem_read), .write_i(pmem_write), .resp_o(pmem_resp), 
    .burst_i(bmem_rdata), .burst_o(bmem_wdata), .address_o(bmem_address), .read_o(bmem_read),
    .write_o(bmem_write), .resp_i(bmem_resp)
    );


    
endmodule : mp3

