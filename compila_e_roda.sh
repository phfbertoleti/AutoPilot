clear

echo "Fazendo make clean..."
make clean

echo "Compilacao em andamento..."
make

echo "Compilacao completa. Projeto entrara em execucao."
sudo ./auto_pilot
