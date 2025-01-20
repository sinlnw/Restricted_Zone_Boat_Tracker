from baseboard import BaseBoard


class Board0(BaseBoard):
    # Note that the BCM numbering for the GPIOs is used.
    DIO0    = 24
    DIO1    = 6
    RST     = 17
    LED     = 4
    SPI_BUS = 0
    SPI_CS  = 8


class Board1(BaseBoard):
    # Note that the BCM numbering for the GPIOs is used.
    DIO0    = 12
    DIO1    = 19
    RST     = 27
    LED     = None
    SPI_BUS = 0
    SPI_CS  = 20
        

class Board2(BaseBoard):
    # Note that the BCM numbering for the GPIOs is used.
    DIO0    = 25
    DIO1    = 13
    RST     = 23
    LED     = 18
    SPI_BUS = 0
    SPI_CS  = 7


class Board3(BaseBoard):
    # Note that the BCM numbering for the GPIOs is used.
    DIO0    = 5
    DIO1    = 26
    RST     = 22
    LED     = None
    SPI_BUS = 0
    SPI_CS  = 21    
