# Tóm tắt
  + Có N hành khách 1, 2, ..., N và M gói hàng N+1,N+ 2,
  ..., N+M. Hành khách (người) (hoặc gói hàng) i có
  điểm đón là i và điểm trả là i+ N + M (i =
  1,2,...,2N+2M).
  + Mỗi gói hàng i có khối lượng qi (i=N+1,...,N+M)
  + Có K taxi 1,...,K xuất phát từ điểm 0. Mỗi xe taxi k có
  thể vận chuyển cùng 1 lúc 1 hành khách và tối đa Qk
  khối lượng hàng (xếp vào cốp xe) (k=1,...,K)
  + Biết rằng d(i,j) là khoảng cách từ điểm i đến điểm j
  (I,j=0,...,2N+2M)
  + Hãy tính toán phương án vận chuyển sao cho tổng
    quãng đường di chuyển của các xe ngắn nhất  

# Input
Cho 1 file txt:
   + Dòng 1 ghi N và M và K
   + Dòng 2 ghi q(N+1), q(N+2), ..., q(N+M)
   + Dòng 3 ghi Q1, Q2, ..., QK
   + Dòng 4 + i (i = 0,1,...,2N+2M) ghi hàng thứ i của ma trận khoảng cách d

# Các thuật toán sử dụng
   + Heuristic Search ( non-optimal)
   + Constraint Programming
   + Mixed Integer Programming

# Cấu trúc
   + Trong directory Project chứa source code, cùng với một vài file txt làm test data
   Big heuristic : Source code của phần heuristic, cho phép người dùng nhập số người, số vật, số xe bất kỳ, từ đó tự generate số liệu. Được sử dụng để thử những case với input lớn ( 100 - 10000)
   + Presentation : Phần thuyết trình của nhóm

# Phần do em đảm nhiệm
  + Tham gia làm cả 3 thuật toán
  + Generate data và phân tích kết quả thực nghiệm
