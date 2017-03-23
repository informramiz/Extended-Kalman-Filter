mkdir build
cd build
cmake .. && make
#./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt ../output/sample-laser-radar-measurement-data-1.txt
./ExtendedKF ../data/sample-laser-radar-measurement-data-2.txt ../output/sample-laser-radar-measurement-data-2.txt
