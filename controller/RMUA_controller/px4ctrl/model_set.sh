cd ./model
rm -r CMakeCache.txt 
cmake .
make 
./nmpc_model_codegen 

