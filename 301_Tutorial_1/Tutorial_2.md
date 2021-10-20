# Tutorial 2: Thiết kế phần cứng và phần mềm

Robot được thiết kế để thiết lập nền tảng robot bốn chân không chổi than hỗ trợ điều khiển lực. Nó có các đặc điểm giá thành thấp, cấu trúc đơn giản, an toàn điện và khả năng mở rộng cao.

## 1. Thiết kế khung phần cứng

Thiết kế của Robot sử dụng 12 động cơ truyền động trực tiếp không chổi than. Ngoài ra, robot cũng bao gồm một bộ điều khiển chính, một nút trình điều khiển, một nút điều khiển pin, một nút bộ xử lý thị giác nhận thức và một không dây nút điều khiển từ xa. Tham khảo Doggo và MIT. Thiết kế phần cứng của rô bốt được thể hiện trong hình sau:

[ Hình ]

Thiết kế nguồn điện đầu vào của rô bốt tương thích 24 ~ 48V. B-G431B-ESC1 được sử dụng như trình điều khiển trong phần drive. Giá trưởng thành cũng tương đối thấp. Cuối cùng, điều khiển từ xa được hiện thực hóa bằng điều khiển từ xa PS2 truyền thống và giao diện điều khiển từ xa SBUS của mẫu máy bay. Ngoài ra, robot sử dụng STM32F4 làm bộ điều khiển. Bộ xử lý hình ảnh vẫn có thể sử dụng ODroid, Raspberry Pi hoặc bộ xử lý dòng Jetson.

Mỗi trình điều khiển giao tiếp với bộ điều khiển chính bằng cổng nối tiếp ở tốc độ truyền 5Mbps. Sử dụng Ethernet làm giao diện truyền thông. Xem xét sử dụng giải pháp MIT để mở rộng cổng CAN với STM32. Nó là cần thiết để thêm một mô-đun nút CAN để giao tiếp với B-G431B-ESC1 thông qua cổng nối tiếp và cuối cùng hoàn thành giao tiếp với bộ điều khiển chính thông qua CAN.

## 2. Thiết kế khung phần mềm

Đối với thiết kế khung phần mềm, để đảm bảo hiệu suất thời gian thực của thuật toán điều khiển và hiệu quả của việc phân bổ tài nguyên trong bộ điều khiển chính, hệ điều hành nhúng hoặc thời gian thực được áp dụng đầu tiên. Hiện tại, UCOSIII và Free-RTOS là thường được sử dụng. Trong hệ thống mà bo mạch là điều khiển chính, cũng cần cài đặt trang điểm thời gian thực RT. Bộ điều khiển chính trước tiên cần thu thập dữ liệu cảm biến điển hình, chẳng hạn như IMU, GPS và nhận dữ liệu góc bộ mã hóa và dòng động cơ của biến tần thông qua CAN dưới dạng phản hồi của lực và động học, đồng thời so với chế độ điều khiển vị trí gửi được Doggo chấp nhận, nó cần phải nhận ra đúng. Bộ điều khiển lực cần đưa ra lệnh dòng điện và mô-men xoắn, vì vậy bộ điều khiển cũng cần hoàn thành việc điều khiển vòng kín vị trí và lực của động cơ ở lớp dưới cùng.

Giao tiếp giữa điều khiển chính và Raspberry Pi có thể được thực hiện bằng cách sử dụng SPI hoặc các cổng nối tiếp. Dữ liệu chính được truyền là ước tính trạng thái và các lệnh điều khiển của lớp cơ thể robot. Trong Raspberry Pi, ROS được sử dụng như một phần mềm vận chuyển để xây dựng một nút điều khiển rô bốt để giải phóng dữ liệu như cảm biến và Gửi SLAM khác, hình ảnh và kết quả điều khiển lập kế hoạch tới rô bốt để thực hiện điều hướng tự động. Tại đây, hãy tham khảo kiến trúc phần mềm của Qualcomm Snapdragon và sửa đổi nó như sau:

[ Hình  ]

Hình trên cho thấy khung phần mềm của Qualcomm Snapdragon Universal Robot RB5. Lớp trên cùng là bộ xử lý lõi của nó. Bởi vì nó sử dụng một SoC hiệu năng cao được tùy chỉnh, cả điều khiển và nhận thức đều được tích hợp trong một con chip duy nhất. Đề xuất sử dụng hai giải pháp để thay thế nó.:

(A) Điều khiển chính một chip + Điều khiển chính một bo mạch Linux: nghĩa là, một sơ đồ tương tự như điều khiển bay Px4 hoặc RobotMaster được sử dụng và một bộ điều khiển chính một chip được sử dụng để hoàn thành tất cả các tính toán thu nhận cảm biến và điều khiển chuyển động để nhận ra điều khiển dáng đi của cơ thể robot. Sử dụng một bộ xử lý chẳng hạn như Bộ xử lý Linux của Jetson hoặc Raspberry Pi để hoàn thành quá trình xử lý hình ảnh cảm nhận và xây dựng một khung phần mềm khung hoàn chỉnh dựa trên ROS;

(B) Điều khiển chính chip đơn Raspberry Pi + Điều khiển chính bo mạch đơn Linux: Sơ đồ trên bị giới hạn bởi sức mạnh tính toán của máy tính vi chip đơn. Nó có thể không đảm bảo chu kỳ điều khiển ổn định khi chạy dáng đi phức tạp. Đồng thời, các thư viện tối ưu hóa như QP không thể được áp dụng. Dựa trên vi điều khiển và Keil để phát triển mã lặp nhanh hơn, giải pháp này tương tự như MIT ngoại trừ việc nó sử dụng Raspberry Pi làm hoạt động điều khiển chuyển động. Sau RT thực- bản vá thời gian được áp dụng, hiệu suất thời gian thực của mã được đảm bảo và cổng SPI hoặc cổng nối tiếp giao tiếp với máy tính chip đơn ở tốc độ cao. Lệnh điều khiển được chuyển đổi thành CAN và được gửi đi và máy tính chip đơn nhận ra bảng mạch sóng mang CAN chuyển và bảng thu nhận cảm biến. Mã có thể được biên dịch và tải xuống nhanh chóng thông qua công nghệ biên dịch chéo;

Đối với hỗ trợ chức năng phần mềm, đề cập đến kiến trúc phần mềm của bộ điều khiển chuyến bay mã nguồn mở và Qualcomm Snapdragon, dự án cuối cùng hy vọng sẽ hoàn thành hỗ trợ kiểm soát chuyển động, nhận thức điều hướng trực quan và triển khai tác nhân AI cho nhiều loại rô bốt. Khung phần cứng có thể tương thích với nhiều loại rô bốt. Cảm biến tầm nhìn, lidar, IMU và GPS thường được sử dụng, với điều hướng tầm nhìn tiêu chuẩn và API điều hướng ROS SLAM, có thể hỗ trợ đầu vào và đầu ra luồng âm thanh và video phổ biến, đồng thời cung cấp SDK người dùng đa ngữ nghĩa toàn diện tương tự cho SpotMini, có thể được thực hiện theo ROS và C ++. Chế độ kiểm soát khung gầm rô bốt và kiểm soát mô-men xoắn cấp thấp của rô bốt cuối cùng cung cấp ứng dụng cấp người dùng APP Demo để tuần tra đường tự động, theo dõi mục tiêu, theo dõi điểm tham chiếu và các nhiệm vụ tự trị cho các tình huống ứng dụng điển hình .

## 3. Thiết kế khung điều khiển

Điều khiển vị trí bộ điều khiển tính toán chiều dài của mỗi chân thông qua phản hồi IMU và tính toán thêm góc của hai động cơ song song thông qua động học và gửi nó đến B-G431B-ESC1 thông qua nối tiếp cổng. B-G431B-ESC1 hoàn thành vòng khép góc ở 10Khz. Điều khiển, phương pháp này không đạt được kiểm soát lực và trình tự giai đoạn của dáng đi cũng rất đơn giản. Không có ước tính hạ cánh thực và ước tính phản hồi lực. Đối với rô bốt điều khiển lực như MIT , nó sẽ gửi một tín hiệu mô-men xoắn đến bộ truyền động, điều khiển GRF của nhà máy. Lực để đạt được điều khiển ổn định. So với phương pháp trên, hiệu suất động và hiệu suất phản ứng của nó cao hơn nhiều. Điều khiển lực cũng là mục tiêu của nền tảng phát triển này. Do đó, tham khảo phương pháp thiết kế kiểm soát lực của MIT, khung kiểm soát được sửa đổi như sau:

[ Hình ]

Như trong hình trên, phương pháp điều khiển cơ bản của rô bốt bốn chân không chổi than có khả năng điều khiển vị trí, điều chỉnh hoạch định quỹ đạo chuyển động của chân, điều này cũng gây ra các khớp có độ cứng cao hoặc không thể đứng vững. Do đó, điều khiển góc khớp Angle được đặt trong liên kết trình điều khiển, vì băng thông cao 10Kz của nó có thể cải thiện đáng kể độ cứng của điều khiển PD. Bộ điều khiển phản hồi giống PID 100Hz được tích hợp trên thân máy để tự động điều chỉnh độ dài chân để giải pháp ở trên mặt đất bằng phẳng và địa hình đơn giản. Ở mức độ trung bình thì không sao, nhưng nếu bạn muốn nhận ra chuyển động ổn định của đống đá trong video, bạn không thể thực hiện được nếu không có lực và kiểm soát mềm.

Cuối cùng, tóm tắt những ưu điểm và nhược điểm của robot bốn chân điều khiển vị trí dựa trên bộ truyền động servo như SimpleFOC, ODrive, v.v. như sau:

(1) Với phản hồi góc (ưu điểm): có thể nhận ra góc quay trở lại, có thể xây dựng một điều khiển độ cứng góc đơn giản và cung cấp một mức độ linh hoạt nhất định;

(2) Băng thông điều khiển cao (ưu điểm): Do băng thông điều khiển của CAN hoặc Uart cao hơn nhiều so với PWM hoặc servo bus nên độ chính xác điều khiển cũng cao;

(3) Không thể kiểm soát lực (nhược điểm): Vì không phải là kiểm soát lực nên cần đảm bảo độ lợi khớp cao khi hỗ trợ khung nặng, và liệu nó có dễ bị rung và lệch khi không có tải hay không, vì vậy lớp dưới cùng cần có tần số điều khiển cao, vì vậy việc điều khiển vị trí chỉ có thể được thực hiện trong vòng lặp FOC của biến tần, và dòng điện không thể được điều khiển thông qua giao tiếp điều khiển chính;

(4) Không thể ước tính lực (nhược điểm): Đối với ODrive, nó có thể cung cấp lại dòng điện của trục Iq, vì vậy nó có thể được sử dụng để ước tính lực chân thực tế GRF. Đối với SimpleFOC, vì phiên bản đầu tiên của nó sử dụng điện áp mở- điều khiển mô-men xoắn vòng, dẫn đến Không thể ước tính mô-men xoắn tải thực. Ngoài ra, do công suất thấp, cần có tỷ số giảm lớn hơn để đảm bảo mô-men xoắn. Bộ truyền động cuối cùng vẫn sẽ giống như một hộp số lái, đó là không dễ tự xử lý, khoảng trống bánh răng, độ trễ giảm tốc, v.v.;
