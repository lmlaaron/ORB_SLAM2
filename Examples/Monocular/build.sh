g++ -std=c++11 -I/usr/local/include -pthread  -c -o mono_kitti.pb.o mono_kitti.pb.cc
g++ -std=c++11 -I/usr/local/include -pthread  -c -o mono_kitti.grpc.pb.o mono_kitti.grpc.pb.cc
g++ -std=c++11 -I/usr/local/include -pthread  -c -o mono_kitti_client.o mono_kitti_client.cc
g++ mono_kitti.pb.o mono_kitti.grpc.pb.o mono_kitti_client.o -L/usr/local/lib `pkg-config --libs grpc++ grpc` -Wl,--no-as-needed -lgrpc++_reflection -Wl,--as-needed -lprotobuf -lpthread -ldl -o mono_kitti_client

