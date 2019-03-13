module fulladder(sum,c_out,a,b,c_in);
output sum,c_out;
input a,b,c_in;

xor X1(o1,a,b);
xor X2(sum,o1,c_in);
and A1(o2,a,b);
and A2(o3,c_in,o1);

or O1(c_out,o2,o3);

endmodule