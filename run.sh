#!/bin/bash
# Di chuyen vao thu muc chua code
cd ~/Desktop/doan/PI5

# Bo tat ca thay doi o local, lay code moi nhat tren server
git reset --hard HEAD
git clean -fd
git pull

rm -rf venv
# Tạo lại venv mới
python3 -m venv venv

# Kích hoạt lại venv
source venv/bin/activate

# Cài lại requirements
pip install -r requirements.txt

# Cai dat cac thu vien can thiet
pip install -r requirements.txt

# Chay chuong trinh python
python app.py