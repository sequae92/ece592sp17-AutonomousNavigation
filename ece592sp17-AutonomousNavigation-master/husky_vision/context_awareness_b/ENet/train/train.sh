#th run.lua --dataset cfl --datapath data/cfl --model models/encoder.lua --save save/cfl-v2/encoder --imHeight 256 --imWidth 512 --labelHeight 32 --labelWidth 64 --nGPU 1 --cachepath save/cfl-v2/encoder/cache 
th run.lua --dataset cfl --datapath data/cfl --model models/decoder.lua --save save/cfl-v2/decoder --imHeight 256 --imWidth 512 --labelHeight 256 --labelWidth 512 --nGPU 1 --cachepath save/cfl-v2/decoder/cache --CNNEncoder save/cfl-v2/encoder/model-best.net -b 8

