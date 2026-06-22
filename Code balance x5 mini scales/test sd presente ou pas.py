import board
import busio
import sdcardio
import storage
import time

try:
    spi = busio.SPI(board.SD_SCK, board.SD_MOSI, board.SD_MISO)
    sd = sdcardio.SDCard(spi, board.SD_CS)
    vfs = storage.VfsFat(sd)
    storage.mount(vfs, "/sd")
    print("SD OK")

    with open("/sd/test.txt", "w") as f:
        f.write("test sd ok")

    print("ECRITURE OK")

except Exception as e:
    print("ERREUR SD :", e)