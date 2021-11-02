# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%                      Smuro Robotics Parameters                      %%%
# %%%                          ZEFFIRETTI, HIESH                          %%%
# %%%                   Beijing Institute of Technology                   %%%
# %%%                zeffiretti@bit.edu.cn, hiesh@mail.com                %%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

from .core import Smuro

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    smuro_para = Smuro(1, 1)
    smuro_para.printInfo()

