const fs=require('fs'),path=require('path');
const {bus,width}=require('./generate_wiring');
const dest=path.join(__dirname,'../tb');fs.mkdirSync(dest,{recursive:true});
fs.writeFileSync(path.join(dest,'logical_bus.vh'),bus.map(([n,w])=>`wire ${width(w)}${n};`).join('\n')+'\n');
const rename={rd_valid:'rd_req_valid',rd_ready:'rd_req_ready',rd_addr:'rd_req_addr',rd_len:'rd_req_len_bytes',rd_tag:'rd_req_tag',
 r_valid:'rd_ret_valid',r_ready:'rd_ret_ready',r_data:'rd_ret_data',r_keep:'rd_ret_keep',r_tag:'rd_ret_tag',r_last:'rd_ret_last',r_error:'rd_ret_error',
 wr_valid:'wr_req_valid',wr_ready:'wr_req_ready',wr_addr:'wr_req_addr',wr_len:'wr_req_len_bytes',wr_tag:'wr_req_tag',
 w_valid:'wr_dat_valid',w_ready:'wr_dat_ready',w_data:'wr_dat_data',w_keep:'wr_dat_keep',w_last:'wr_dat_last',
 b_valid:'wr_cplt_valid',b_ready:'wr_cplt_ready',b_tag:'wr_cplt_tag',b_error:'wr_cplt_error'};
fs.writeFileSync(path.join(dest,'logical_memory.vh'),`wire [15:0] violations;
ddr_memory_model memory(.clk(clk),.rst_n(rst_n),.proto_violations(violations),
 ${bus.map(([n])=>`.${rename[n]}(${n})`).join(',\n ')});
`);
fs.writeFileSync(path.join(dest,'logical_connections.vh'),bus.map(([n])=>`.${n}(${n})`).join(',\n')+'\n');
