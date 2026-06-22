import board

for attr in dir(board):
    if "SCL" in attr or "SDA" in attr:
        print(attr)