# moduleCNC

3. Module nội suy cho máy CNC, robot
•	Đầu vào clk (1us), WR, LS
•	Đầu vào N [7:0], Nx [7:0]
•	Đầu ra Pulse, Dir
•	Khi có xung cạnh lên của WR, Nx sẽ nạp giá trị mới.
•	Khi LS = 1, ngõ ra Pulse = 0, LS = 0, Pulse xuất ra theo nội suy.
•	Nx [7] qui định bit dấu cho Dir, Nx [7] = 1, Dir = 1.
•	Nx [7] = 0, Dir = 0.
•	Nx [6:0] là giá trị số xung cần xuất
Yêu cầu : 
1. Viết cho 2 trục X,Y 
2. Khi có xung cạnh lên của WR, Nx nạp giá trị mới. Nhưng giá trị này không tác động ở 
chu kỳ T hiện tại, mà tác động ở chu kỳ T kế tiếp. 
3. Khi không ghi giá trị Nx (không xuất xung WR và Nbuff = 0) ở chu kỳ T hiện tại, chu 
kỳ T kế tiếp không được xuất xung ra chân Pulse. (Không được lấy giá trị cũ để rải tiếp) 
4. Khi mô phỏng, add thêm 1 ngõ ra flag_T để kiểm tra hết mỗi chu kỳ T. 
5. Mô phỏng N = 10, Cố định (k đổi N) 
6. clk = 1us, clk1 = 100us, 
Thuật toán xuất xung dựa vào clk1, các tín hiệu khác dựa vào clk. 
7. Cho phép ghi tối đa 4 xung WR vào buffer trong 1 chu kỳ flag_T (Nbuff = 4), không 
lưu buffer nếu xung WR thứ 5 xảy ra. Khi ghi đủ 4 giá trị cờ flag_full = 1. Kiểm tra chỉ 
khi flag_full = 0 (khi Nbuff = 3, 2, 1, 0) thì mới tiếp tục nhận giá trị WR. 
8. Khi LS = 1, ngõ ra Pulse = 0 và tất cả buffer sẽ bị xóa, xem như trạng thái reset lại từ 
đầu. Pulse chỉ xuất ở chu kỳ T kế tiếp khi LS = 0 và có xung WR sau đó.
3.1. Nguyên lí hoạt động:
-Ta dụng thuật toán nội suy đường thẳng theo phương pháp xung chuẩn.
  
- Khi có xung cạnh lên của WR, Nx nạp giá trị mới nhưng giá trị này không động ở chu kỳ T hiện tại, mà tác động ở chu kỳ T kế tiếp (không được lấy giá trị cũ tác để rải tiếp).
-Ta sử dụng thêm 1 ngõ ra flag_T để kiểm tra hết mỗi chu kỳ T.
-Khi LS = 1, ngõ ra Pulse = 0, LS = 0, Pulse xuất ra theo nội suy.
- Nx[7] qui định bit dấu cho Dir, Nx[7] = 1, Dir = 1, Nx [7] = 0, Dir = 0 ;Nx[6:0] là giá trị số xung cần xuất.
-Cho phép buffer tối đa 4 xung WR trong 1 chu kỳ flag_T, không lưu buffer nếu xung
WR thứ 5 xảy ra, Nbuff = 4. Khi ghi đủ 4 giá trị cờ f_full = 1. Kiểm tra chỉ khi flag_full = 0 (khi Nbuff = 3, 2, 1, 0) mới được WR tiếp.
3.2. Code:
 module CNC(clk,WR,LS,Nx,N,Pulse_x,Pulse_y,Dir_x,T_flag,flag_full,Ny,Dir_y); // Px so xung truc X, Py so xung truc y 
input clk,WR,LS; 							//Canh len WR, Nx nap gia tri moi, ko tac dong o chu ky T_flag hien tai  
input [7:0]N,Nx,Ny;							//LS=1 Pulse=0
output reg Pulse_x,Pulse_y,Dir_x,Dir_y,T_flag,flag_full;
reg clk1,pinout,pre_WR,limit,start,pinout1;
reg [7:0]acc,tmp_cky,acc1;
reg [5:0]temp_clk=0;
reg [7:0]Nx_0,Nx_1,Nx_2,Nx_3,Nx_4;	
reg [7:0]Ny_0,Ny_1,Ny_2,Ny_3,Ny_4;	
reg [2:0] Nbuff,Nbuff1;	
initial 
begin
	start=1;
	flag_full=0;
end
always@(posedge clk1) begin
	if(start) begin 
		acc=N;
		start=0;
		acc1=acc;
	end
	else begin
		acc=acc+Nx_0[6:0];
		acc1=acc1+Ny_0[6:0];
		if(acc>N) begin
			acc=acc-N; pinout=1;
		end
		else begin 
			pinout=0; 
		end
		if(acc1>N) begin
			acc1=acc1-N; pinout1=1;
		end
		else begin
			pinout1=0;
		end
	end
end 
always@ (posedge clk) begin
	pre_WR<=WR;
	
	if (tmp_cky<N ) begin
		//xuat Pulse LS=1 Pulse=0
		if (LS) begin 
			Nbuff<=0;
			Pulse_x<=0;
			Pulse_y<=0;
			flag_full<=0;
			Nx_0<=0;
			Nx_1<=0;
			Nx_2<=0;
			Nx_3<=0;
			Ny_0<=0;
			Ny_1<=0;
			Ny_2<=0;
			Ny_3<=0;
		end
		else begin
			if(limit) begin 
				Pulse_x<=0;
				Pulse_y<=0;
			end
			else begin
				Pulse_x=clk1&pinout;
				Pulse_y=clk1&pinout1;
			end
			// create clk1 T_clk1=100us
			if (temp_clk==50) begin
				temp_clk<=0;
				if(clk1) begin
					clk1<=0;
					tmp_cky<=tmp_cky+1;
				end
				else 
					clk1<=1;
			end
			else temp_clk<=temp_clk+1;
			
			// Posedge WR luu trong buffer Nx_1,2,3,4, flag full thi khong cho nap
			if({pre_WR,WR}==2'b01 && !flag_full  ) begin
				Nbuff<=Nbuff+1;
				case (Nbuff)
					0: 	begin
							Nx_1<=Nx;
							Ny_1<=Ny;
						end
					1: 	begin
							Nx_2<=Nx;
							Ny_2<=Ny;
						end
					2: 	begin
							Nx_3<=Nx;
							Ny_3<=Ny;
						end
					3: 	begin
							Nx_4<=Nx;
							Ny_4<=Ny;
							flag_full<=1;
						end
				endcase
			end
		end
	end
	// khi du 10 chu ky clk1 thi cap nhat 1 lan
	else if(tmp_cky==N) begin
		tmp_cky<=0;
		Nx_0<=Nx_1;
		Nx_1<=Nx_2;
		Nx_2<=Nx_3;
		Nx_3<=Nx_4;
		Ny_0<=Ny_1;
		Ny_1<=Ny_2;
		Ny_2<=Ny_3;
		Ny_3<=Ny_4;
		Dir_x<=Nx_1[7];
		Dir_y<=Ny_1[7];
		flag_full<=0;
		if(Nbuff==0) limit<=1;
		else begin
			Nbuff<=Nbuff-1;
			limit=0;
		end
		if(T_flag)
			T_flag<=0;
		else 
			T_flag<=1;
		end
end

endmodule
	
 
 
 
3.3. Mô phỏng với N = 10
Ngõ vào xung clk với chu kì 1 us 
Kết quả mô phỏng thông thường: 
 
Khi có tín hiệu LS:
 
3.4. Nhận xét:
Chương trình mô phỏng chính xác module nội suy cho máy CNC, robot; khi LS = 1, ngõ ra Pulse = 0 và chỉ khi LS = 0, Pulse xuất ra theo nội suy. Khi có xung cạnh lên của WR,
Nx sẽ nạp giá trị mới giá trị này không tác động ở chu kỳ T hiện tại, mà tác động ở chu kỳ T kế tiếp. Ngõ ra f_full bằng 1 khi bộ đếm buff đủ 4 xung WR và không nhận xung WR thứ 5, chỉ khi bộ đếm buff có giá trị là 0,1,2,3 thì mới tiếp tục nhận xung WR.
 
 

