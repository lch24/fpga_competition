// Validate capacities and 33-bit address arithmetic before any DDR activity.
// capacity describes bytes available starting at each base. RGB565 rows are
// 2*width bytes; maps contain 4*width bytes per row. Padding is not written.
module descriptor_check #(parameter MAX_W=1920,MAX_H=1080) (
 input wire build_map,input wire [15:0] width,height,
 input wire [31:0] src_base,dst_base,map_x_base,map_y_base,
 input wire [31:0] src_stride,dst_stride,map_stride,
 input wire [31:0] src_capacity,dst_capacity,map_capacity,
 output wire valid
);
 wire [32:0] image_row={16'd0,width,1'b0},map_row={15'd0,width,2'b00};
 wire [63:0] ss=({48'd0,height}-1)*src_stride+image_row;
 wire [63:0] ds=({48'd0,height}-1)*dst_stride+image_row;
 wire [63:0] ms=({48'd0,height}-1)*map_stride+map_row;
 wire [63:0] se={32'd0,src_base}+ss,de={32'd0,dst_base}+ds;
 wire [63:0] xe={32'd0,map_x_base}+ms,ye={32'd0,map_y_base}+ms;
 wire images_disjoint=(se<=dst_base||de<=src_base);
 wire maps_disjoint=(xe<=map_y_base||ye<=map_x_base);
 wire maps_images_disjoint=(xe<=src_base||se<=map_x_base)&&
  (ye<=src_base||se<=map_y_base)&&(xe<=dst_base||de<=map_x_base)&&
  (ye<=dst_base||de<=map_y_base);
 assign valid=width>0&&height>0&&width<=MAX_W&&height<=MAX_H&&
  map_stride>=map_row&&ms<=map_capacity&&xe<=64'h40000000&&ye<=64'h40000000&&maps_disjoint&&
  (build_map||(src_stride>=image_row&&dst_stride>=image_row&&ss<=src_capacity&&ds<=dst_capacity&&
   se<=64'h40000000&&de<=64'h40000000&&images_disjoint&&maps_images_disjoint));
endmodule
