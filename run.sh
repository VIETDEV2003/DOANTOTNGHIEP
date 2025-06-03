#!/bin/bash
# Di chuyen vao thu muc chua code
cd ~/Desktop/doan/PI5

# Bo tat ca thay doi o local, lay code moi nhat tren server
git reset --hard HEAD
git clean -fd
git pull


# Kich hoat moi truong ao python
source venv/bin/activate

# Chay chuong trinh python
python app.py