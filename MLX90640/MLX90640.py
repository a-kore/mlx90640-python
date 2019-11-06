from smbus2 import SMBus, i2c_msg
import math
import time
import numpy as np
import cv2

class MLX90640:

    def __init__(self, address=0x33):    #default 0x33
        self.bus = SMBus(1)
        self.addr = address
        self.eeData = self.dumpEE()
        self.resolutionEE = self.ExtractResolutionParameters()
        self.tgc = self.ExtractTgcParameters()
        self.KsTa = self.ExtractKsTaParameters()
        self.ExtractVDDParameters()
        self.ExtractGainParameters()
        self.ExtractKsToParameters()
        self.ExtractPTATParameters()
        self.ExtractCPParameters()
        self.ExtractCILCParameters()
        self.ExtractOffsetParameters()
        self.ExtractKtaPixelParameters()
        self.ExtractKvPixelParameters()
        self.ExtractAlphaParameters()
        self.ExtractDeviatingPixels()
        self.SetResolution(0b100)
    
    def setRegf(self, reg, value):
        write = i2c_msg.write(0x33,[reg>>8,reg&0xFF, value>>8, value&0xFF])
        self.bus.i2c_rdwr(write)
        return 0

    
    def getRegs(self,reg,num):
        p = [0]*num
        write = i2c_msg.write(self.addr,[reg>>8,reg&0xFF])
        read = i2c_msg.read(self.addr,num*2)
        self.bus.i2c_rdwr(write, read)
        temp = list(read)
        for count in range(num):
            i = count << 1
            p[count] = (temp[i] << 8) | temp[i+1]
        return p
    
    
    def getRegf(self,reg):
        write = i2c_msg.write(self.addr,[reg>>8,reg&0xFF])
        read = i2c_msg.read(self.addr,2)
        self.bus.i2c_rdwr(write, read)
        result = list(read)
        return (result[0]<<8)+result[1]

    
    def dumpEE(self):
        return self.getRegs(0x2400,832)
    
    
    def ExtractVDDParameters(self):
        
        kVdd = (self.eeData[51] & 0xFF00) >> 8
        if kVdd > 127:
            kVdd = kVdd - 256
            
        kVdd = 32 * kVdd
        vdd25 = self.eeData[51] & 0x00FF
        vdd25 = ((vdd25 - 256) << 5) - 8192
        
        self.kVdd = kVdd
        self.vdd25 = vdd25
        
        return 0    
    
    
    def ExtractPTATParameters(self):
        
        KvPTAT = (self.eeData[50] & 0xFC00) >> 10
        if KvPTAT > 31:
            KvPTAT = KvPTAT - 64
        KvPTAT = KvPTAT/4096
        
        KtPTAT = self.eeData[50] & 0x03FF
        if KtPTAT > 511:
            KtPTAT = KtPTAT - 1024
        KtPTAT = KtPTAT/8
        
        vPTAT25 = self.eeData[49]
        
        alphaPTAT = (self.eeData[16] & 0xF000) /  pow(2, 14) + 8.0
        
        self.KvPTAT = KvPTAT
        self.KtPTAT = KtPTAT
        self.vPTAT25 = vPTAT25
        self.alphaPTAT = alphaPTAT
        
        return 0    
    
    
    def ExtractGainParameters(self):
        
        gainEE = self.eeData[48]
        if gainEE > 32767:
            gainEE = gainEE - 65536
                
        self.gainEE = gainEE
        
        return 0    

    
    def ExtractTgcParameters(self):

        tgc = self.eeData[60] & 0x00FF;
        if(tgc > 127):
            tgc = tgc - 256
        tgc = tgc / 32.0

        return tgc    
    
    
    def ExtractResolutionParameters(self):
        resolutionEE = (self.eeData[56] & 0x3000) >> 12
        return resolutionEE

    
    def ExtractKsTaParameters(self):

        KsTa = (self.eeData[60] & 0xFF00) >> 8
        if(KsTa > 127):
            KsTa = KsTa -256
        KsTa = KsTa / 8192.0
        
        return KsTa
    
    def ExtractKsToParameters(self):

        self.ct = [0]*4
        self.ksTo = [0]*4
        step = ((self.eeData[63] & 0x3000) >> 12) * 10

        self.ct[0] = -40
        self.ct[1] = 0
        self.ct[2] = (self.eeData[63] & 0x00F0) >> 4
        self.ct[3] = (self.eeData[63] & 0x0F00) >> 8

        self.ct[2] = self.ct[2]*step
        self.ct[3] = self.ct[2] + self.ct[3]*step

        KsToScale = (self.eeData[63] & 0x000F) + 8
        KsToScale = 1 << KsToScale

        self.ksTo[0] = self.eeData[61] & 0x00FF
        self.ksTo[1] = (self.eeData[61] & 0xFF00) >> 8
        self.ksTo[2] = self.eeData[62] & 0x00FF
        self.ksTo[3] = (self.eeData[62] & 0xFF00) >> 8


        for i in range(3):
        
            if(self.ksTo[i] > 127):
            
                self.ksTo[i] = self.ksTo[i] -256
            
            self.ksTo[i] = self.ksTo[i] / KsToScale

        return 0
    
    def ExtractAlphaParameters(self):
        
        self.alpha = [0]*768
        accRow = [0]*24
        accColumn  = [0]*32


        accRemScale = self.eeData[32] & 0x000F
        accColumnScale = (self.eeData[32] & 0x00F0) >> 4
        accRowScale = (self.eeData[32] & 0x0F00) >> 8
        alphaScale = ((self.eeData[32] & 0xF000) >> 12) + 30
        alphaRef = self.eeData[33]

        for i in range(6):
            p = i * 4
            accRow[p + 0] = (self.eeData[34 + i] & 0x000F)
            accRow[p + 1] = (self.eeData[34 + i] & 0x00F0) >> 4
            accRow[p + 2] = (self.eeData[34 + i] & 0x0F00) >> 8
            accRow[p + 3] = (self.eeData[34 + i] & 0xF000) >> 12
            
        for i in range(24):
            if (accRow[i] > 7):
                accRow[i] = accRow[i] - 16

        for i in range(8):
            p = i * 4
            accColumn[p + 0] = (self.eeData[40 + i] & 0x000F)
            accColumn[p + 1] = (self.eeData[40 + i] & 0x00F0) >> 4
            accColumn[p + 2] = (self.eeData[40 + i] & 0x0F00) >> 8
            accColumn[p + 3] = (self.eeData[40 + i] & 0xF000) >> 12

        for i in range(32):
            if (accColumn[i] > 7):
                accColumn[i] = accColumn[i] - 16
                
        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                self.alpha[p] = (self.eeData[64 + p] & 0x03F0) >> 4
                if (self.alpha[p] > 31):
                    self.alpha[p] = self.alpha[p] - 64
                self.alpha[p] = self.alpha[p]*(1 << accRemScale)
                self.alpha[p] = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + self.alpha[p])
                self.alpha[p] = self.alpha[p] / pow(2,alphaScale)
        
        return 0 
            
   
    def ExtractOffsetParameters(self):
        
        self.offset = [0]*768
        occRow = [0]*24
        occColumn = [0]*32
        
        occRemScale = (self.eeData[16] & 0x000F)
        occColumnScale = (self.eeData[16] & 0x00F0) >> 4
        occRowScale = (self.eeData[16] & 0x0F00) >> 8
        offsetRef = self.eeData[17]
        if (offsetRef > 32767):
            offsetRef = offsetRef - 65536
        

        for i in range(6):
            p = i * 4 
            occRow[p + 0] = (self.eeData[18 + i] & 0x000F) 
            occRow[p + 1] = (self.eeData[18 + i] & 0x00F0) >> 4 
            occRow[p + 2] = (self.eeData[18 + i] & 0x0F00) >> 8 
            occRow[p + 3] = (self.eeData[18 + i] & 0xF000) >> 12 


        for i in range(24):
            if occRow[i] > 7:
                occRow[i] = occRow[i] - 16 

        for i in range(8):
            p = i * 4 
            occColumn[p + 0] = (self.eeData[24 + i] & 0x000F) 
            occColumn[p + 1] = (self.eeData[24 + i] & 0x00F0) >> 4 
            occColumn[p + 2] = (self.eeData[24 + i] & 0x0F00) >> 8 
            occColumn[p + 3] = (self.eeData[24 + i] & 0xF000) >> 12 

        for i in range(32):
            if occColumn[i] > 7:
                occColumn[i] = occColumn[i] - 16 

        for i in range(24):
            for j in range(32):
                p = 32 * i +j 
                self.offset[p] = (self.eeData[64 + p] & 0xFC00) >> 10 
                if (self.offset[p] > 31):
                    self.offset[p] = self.offset[p] - 64 
                
                self.offset[p] =self.offset[p]*(1 << occRemScale) 
                self.offset[p] = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + self.offset[p]) 
    
        return 0
    
    
    def ExtractKtaPixelParameters(self):
        
        self.kta = [0]*768
        KtaRC = [0]*4
        
        KtaRoCo = (self.eeData[54] & 0xFF00) >> 8
        if (KtaRoCo > 127):
            KtaRoCo = KtaRoCo - 256
        KtaRC[0] = KtaRoCo

        KtaReCo = (self.eeData[54] & 0x00FF)
        if (KtaReCo > 127):
            KtaReCo = KtaReCo - 256
        KtaRC[2] = KtaReCo

        KtaRoCe = (self.eeData[55] & 0xFF00) >> 8
        if (KtaRoCe > 127):
            KtaRoCe = KtaRoCe - 256
        KtaRC[1] = KtaRoCe

        KtaReCe = (self.eeData[55] & 0x00FF)
        if (KtaReCe > 127):
            KtaReCe = KtaReCe - 256
        KtaRC[3] = KtaReCe

        ktaScale1 = ((self.eeData[56] & 0x00F0) >> 4) + 8
        ktaScale2 = (self.eeData[56] & 0x000F)

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                split = int(2*(p/32 - (p/64)*2) + p%2)
                self.kta[p] = (self.eeData[64 + p] & 0x000E) >> 1
                if (self.kta[p] > 3):
                    self.kta[p] = self.kta[p] - 8
                self.kta[p] = self.kta[p] * (1 << ktaScale2)
                self.kta[p] = KtaRC[split] + self.kta[p]
                self.kta[p] = self.kta[p] / pow(2, ktaScale1)
                
        return 0

    
    def ExtractKvPixelParameters(self):
        
        self.kv = [0]*768
        KvT = [0]*4
        
        KvRoCo = (self.eeData[52] & 0xF000) >> 12
        if (KvRoCo > 7):
            KvRoCo = KvRoCo - 16
        KvT[0] = KvRoCo

        KvReCo = (self.eeData[52] & 0x0F00) >> 8
        if (KvReCo > 7):
            KvReCo = KvReCo - 16
        KvT[2] = KvReCo

        KvRoCe = (self.eeData[52] & 0x00F0) >> 4
        if (KvRoCe > 7):
            KvRoCe = KvRoCe - 16
        KvT[1] = KvRoCe

        KvReCe = (self.eeData[52] & 0x000F)
        if (KvReCe > 7):
            KvReCe = KvReCe - 16
        KvT[3] = KvReCe

        kvScale = (self.eeData[56] & 0x0F00) >> 8


        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                split = int(2*(p/32 - (p/64)*2) + p%2)
                self.kv[p] = KvT[split];
                self.kv[p] = self.kv[p] / pow(2,kvScale);
                
        return 0   
    
    
    def ExtractCPParameters(self):
        self.cpAlpha = [0,0]
        self.cpOffset = [0,0]
        alphaSP = [0,0]
        offsetSP = [0,0]
        
        alphaScale = ((self.eeData[32] & 0xF000) >> 12) + 27
        
        offsetSP[0] = (self.eeData[58] & 0x03FF)
        if offsetSP[0] > 511:
            offsetSP[0] = offsetSP[0] - 1024
        
        offsetSP[1] = (self.eeData[58] & 0xFC00) >> 10
        if offsetSP[1] > 31:
            offsetSP[1] = offsetSP[1] - 64
            
        offsetSP[1] = offsetSP[1] + offsetSP[0]
        
        alphaSP[0] = (self.eeData[57] & 0x03FF)
        if alphaSP[0] > 511:
            alphaSP[0] = alphaSP[0] - 1024
        
        alphaSP[0] = alphaSP[0] / pow(2, alphaScale)
        
        alphaSP[1] = (self.eeData[57] & 0xFC00) >> 10
        if alphaSP[1] > 31:
            alphaSP[1] = alphaSP[1] - 64
            
        alphaSP[1] = (1+ alphaSP[1]/128) * alphaSP[0]
        
        cpKta = (self.eeData[59] & 0x00FF)
        if cpKta > 127:
            cpKta = cpKta - 256
        
        ktaScale1 = ((self.eeData[56] & 0x00F0) >> 4) + 8
        self.cpKta = cpKta / pow(2, ktaScale1)
        
        cpKv = (self.eeData[59] & 0xFF00) >> 8
        if cpKv > 127:
            cpKv = cpKv - 256
        
        kvScale = (self.eeData[56] & 0x0F00) >> 8
        self.cpKv = cpKv / pow(2, kvScale)
        
        self.cpAlpha[0] = alphaSP[0]
        self.cpAlpha[1] = alphaSP[1]
        self.cpOffset[0] = offsetSP[0]
        self.cpOffset[1] = offsetSP[1]
        
        return 0
    
    def ExtractCILCParameters(self):
        
        self.ilChessC = [0,0,0]
        ilChessSP = [0,0,0]
        
        calibrationModeEE = (self.eeData[10] & 0x0800) >> 4
        calibrationModeEE = calibrationModeEE ^ 0x80
        
        ilChessSP[0] = (self.eeData[53] & 0x003F)
        if ilChessSP[0] > 31:
            ilChessSP[0] = ilChessSP[0] - 64
        ilChessSP[0] = ilChessSP[0] / 16.0
        
        ilChessSP[1] = (self.eeData[53] & 0x07C0) >> 6
        if ilChessSP[1] > 15:
            ilChessSP[1] = ilChessSP[1] - 32
        ilChessSP[1] = ilChessSP[1] / 2.0
        
        ilChessSP[2] = (self.eeData[53] & 0xF800) >> 11
        if (ilChessSP[2] > 15):
            ilChessSP[2] = illChessSP[2] - 32
        ilChessSP[2] = ilChessSP[2] / 8.0
        
        self.calibrationModeEE = calibrationModeEE
        self.ilChessC[0] = ilChessSP[0]
        self.ilChessC[1] = ilChessSP[1]
        self.ilChessC[2] = ilChessSP[2]
        
        return 0
    
    def SetDeviceMode(self, deviceMode):
    
        value = (deviceMode & 0x01)<<4
        ctrl_reg = self.getRegf(0x800D)
        value = (ctrl_reg & 0b1111111111111101) | value
        self.setRegf(0x800D, value) 
    
        return value
    
    def SetSubPageRepeat(self, subPageRepeat):

        value = (subPageRepeat & 0x01)<<3
        ctrl_reg = self.getRegf( 0x800D)
        value = (ctrl_reg & 0b1111111111110111) | value
        self.setRegf(0x800D, value)
        
        return value
    
    
    def SetRefreshRate(self, refreshRate):

        value = (refreshRate & 0x07)<<7
        ctrl_reg = self.getRegf(0x800D)
        value = (ctrl_reg & 0xFC7F) | value
        self.setRegf(0x800D, value) 

        return value
    

    def GetRefreshRate(self):

        ctrl_reg = self.getRegf(0x800D)
        refreshRate = (ctrl_reg & 0x0380) >> 7

        return refreshRate
    

    def SetChessMode(self):
        ctrl_reg = self.getRegf(0x800D)
        set_chess = (ctrl_reg | 0x1000)
        self.setRegf(0x800D, set_chess)

        return set_chess
    
    
    def SetResolution(self, resolution):
    
        value = (resolution & 0x03) << 10;

        ctrl_reg = self.getRegf(0x800D)

        value = (ctrl_reg & 0xF3FF) | value;
        self.setRegf(0x800D, value)        
  

        return value
    


    def GetCurResolution(self):
    

        ctrl_reg = self.getRegf(0x800D)
        resolutionRAM = (ctrl_reg & 0x0C00) >> 10;

        return resolutionRAM
    

    def ExtractDeviatingPixels(self):
        
        self.brokenPixels = [0]*768
        self.outlierPixels = [0]*768
        eeData = self.eeData
        pixCnt = 0;
        brokenPixCnt = 0;
        outlierPixCnt = 0;
        warn = 0;
        

        for pixCnt in range(5):
       
            self.brokenPixels[pixCnt] = 0xFFFF
            self.outlierPixels[pixCnt] = 0xFFFF
        

        pixCnt = 0    
        while (pixCnt < 768 and brokenPixCnt < 5 and outlierPixCnt < 5):
        
            if(eeData[pixCnt+64] == 0):
            
                self.brokenPixels[brokenPixCnt] = pixCnt
                brokenPixCnt = brokenPixCnt + 1
               
            elif((eeData[pixCnt+64] & 0x0001) != 0):
            
                self.outlierPixels[outlierPixCnt] = pixCnt
                outlierPixCnt = outlierPixCnt + 1
                

            pixCnt = pixCnt + 1

        

        if(brokenPixCnt > 4):  
        
            warn = -3
                 
        elif(outlierPixCnt > 4):  
        
            warn = -4
        
        elif((brokenPixCnt + outlierPixCnt) > 4):  
        
            warn = -5
         
        else:
        
            for pixCnt in range(brokenPixCnt):
            
                for i in range(pixCnt+1,brokenPixCnt):
                
                    warn = CheckAdjacentPixels(self.brokenPixels[pixCnt],self.brokenPixels[i])
                    if(warn != 0):
                    
                        return warn
                        
                    
            

            for pixCnt in range(outlierPixCnt):
            
                for i in range(pixCnt+1,outlierPixCnt):
                
                    warn = CheckAdjacentPixels(self.outlierPixels[pixCnt],self.outlierPixels[i])
                    if(warn != 0):
                    
                        return warn
                        
                    
             

            for pixCnt in range(brokenPixCnt):
            
                for i in range(outlierPixCnt):
                
                    warn = CheckAdjacentPixels(self.brokenPixels[pixCnt],self.outlierPixels[i])
                    if(warn != 0):
                    
                        return warn
                        
        return warn;

    
    
    
    def CheckAdjacentPixels(self, pix1, pix2):
     

        pixPosDif = pix1 - pix2
        
        if(pixPosDif > -34 and pixPosDif < -30):

            return -6

        if(pixPosDif > -2 and pixPosDif < 2):

            return -6

        if(pixPosDif > 30 and pixPosDif < 34):

            return -6


        return 0    
      

    def GetMedian(self, values, n):

        for i in range(n-1):

            for j in range(i+1,n):

                if(values[j] < values[i]): 
             
                    temp = values[i]
                    values[i] = values[j]
                    values[j] = temp
                    
        if(n%2==0): 
            return ((values[n/2] + values[n/2 - 1]) / 2.0)
        else: 
            return values[n/2]
     

    def IsPixelBad(self, pixel):
    
        for i in range(5):
            if(pixel == self.outlierPixels[i] or pixel == self.brokenPixels[i]):
                return 1;

        return 0;     
    
    
    def BadPixelsCorrection(self, pixels, to, mode):
     
        ap = [0]*4

        pix = 0
        while(pixels[pix]< 0xFFFF):
       
            line = pixels[pix]>>5
            column = pixels[pix] - (line<<5)

            if(mode == 1):
                    
                if(line == 0):
                
                    if(column == 0):
                           
                        to[pixels[pix]] = to[33]                    
                    
                    elif(column == 31):
                    
                        to[pixels[pix]] = to[62]                     
                    
                    else:
                    
                        to[pixels[pix]] = (to[pixels[pix]+31] + to[pixels[pix]+33])/2.0                    
                            
                
                elif(line == 23):
                
                    if(column == 0):
                    
                        to[pixels[pix]] = to[705]                    
                    
                    elif(column == 31):
                    
                        to[pixels[pix]] = to[734];                       
                    
                    else:
                    
                        to[pixels[pix]] = (to[pixels[pix]-33] + to[pixels[pix]-31])/2.0;                       
                                           
                 
                elif(column == 0):
                
                    to[pixels[pix]] = (to[pixels[pix]-31] + to[pixels[pix]+33])/2.0;                
                
                elif(column == 31):
                
                    to[pixels[pix]] = (to[pixels[pix]-33] + to[pixels[pix]+31])/2.0;                
                 
                else:
                
                    ap[0] = to[pixels[pix]-33];
                    ap[1] = to[pixels[pix]-31];
                    ap[2] = to[pixels[pix]+31];
                    ap[3] = to[pixels[pix]+33];
                    to[pixels[pix]] = GetMedian(ap,4)
                                   
            
            else:
                   
                if(column == 0):
                
                    to[pixels[pix]] = to[pixels[pix]+1];            
                
                elif(column == 1 or column == 30):
                
                    to[pixels[pix]] = (to[pixels[pix]-1]+to[pixels[pix]+1])/2.0;                
                 
                elif(column == 31):
                
                    to[pixels[pix]] = to[pixels[pix]-1];
                
                else:
                
                    if(IsPixelBad(pixels[pix]-2,params) == 0 and IsPixelBad(pixels[pix]+2,params) == 0):
                    
                        ap[0] = to[pixels[pix]+1] - to[pixels[pix]+2]
                        ap[1] = to[pixels[pix]-1] - to[pixels[pix]-2]
                        if(fabs(ap[0]) > fabs(ap[1])):
                        
                            to[pixels[pix]] = to[pixels[pix]-1] + ap[1]                     
                        
                        else:
                        
                            to[pixels[pix]] = to[pixels[pix]+1] + ap[0]                       
                        
                    
                    else:
                    
                        to[pixels[pix]] = (to[pixels[pix]-1]+to[pixels[pix]+1])/2.0;                    
                                
                                     
             
            pix = pix + 1;    
        
        return to
    
    def InterpolateOutliers(self, frameData):
        
        eepromData = self.eeData
    
        for x in range(768):
            broken = eepromData[64 + x] == 0
            outlier = eepromData[64 + x] & 0x0001
            if (broken):
                val = 0
                count = 0
                if(x - 33 > 0):
                    val += frameData[x - 33]
                    val += frameData[x - 31]
                    count += 2
                elif (x - 31 > 0):
                    val += frameData[x - 31];
                    count += 1;
                
                if(x + 33 < 768):
                    val += frameData[x + 33];
                    val += frameData[x + 31];
                    count += 2;
                elif (x + 31 < 768):
                    val += frameData[x + 31];
                    count += 1;
                
                frameData[x] = (uint16_t)((float)(val / count) * 1.0003);
            
        

        return frameData
    

    
    def GetVDD(self, frame):

        vdd = frame[810]
        if vdd > 32767:
            vdd = vdd - 65536

        resolutionRAM = (frame[832] & 0x0c00) >> 10
        resolutionCorrection = pow(2, self.resolutionEE) / pow(2, resolutionRAM)
        vdd = (resolutionCorrection * vdd - self.vdd25) / (self.kVdd + 3.3)

        return vdd

    def GetTa(self, frame):
        vdd = self.GetVDD(frame)
        ptat = frame[800]
        if ptat > 32767:
            ptat = ptat - 65536;

        ptatArt = frame[768]
        if ptatArt > 32767:
            ptatArt = ptatArt - 65536

        ptatArt = (ptat / (ptat * self.alphaPTAT + ptatArt )) * pow(2, 18)

        ta = (ptatArt / (1 + self.KvPTAT * (vdd - 3.3)) - self.vPTAT25)
        ta = ta / self.KtPTAT + 25

        return ta
    
    def CalculateTo(self, frame, emissivity):
        result = [0]*768

        irDataCP = [0,0]
        alphaCorrR = [0, 0, 0, 0]
        KsTo = self.ksTo
        ct = self.ct
        subPage = frame[833]
        vdd = self.GetVDD(frame)
        ta = self.GetTa(frame)
        tr = ta - 8
        ta4 = pow((ta + 273.15), 4)
        tr4 = pow((tr + 273.15), 4)
        taTr = tr4 - (tr4-ta4)/emissivity
        alphaCorrR[0] = 1/ (1 + KsTo[0] * 40)
        alphaCorrR[1] = 1
        alphaCorrR[2] = (1 + KsTo[1] * ct[1])
        alphaCorrR[3] = (1 + KsTo[2] * (ct[3] - ct[2]))

        #gain calculation

        gain = frame[778]
        if gain > 32767:
            gain = gain - 65536

        gain = self.gainEE / gain

        #To calculation

        mode = (frame[832] & 0x1000) >> 5

        irDataCP[0] = frame[776]
        irDataCP[1] = frame[808]
        for i in range(2):
            if irDataCP[i] > 32767:
                irDataCP[i] = irDataCP[i] - 65536

            irDataCP[i] = irDataCP[i] * gain

        irDataCP[0] = irDataCP[0] - self.cpOffset[0] * (1 + self.cpKta * (ta -25)) * (1 + self.cpKv * (vdd -3.3))
        if mode == self.calibrationModeEE:
            irDataCP[1] = irDataCP[1] - self.cpOffset[1] * (1 + self.cpKta * (ta -25)) * (1 + self.cpKv * (vdd -3.3))

        else:
            irDataCP[1] = irDataCP[1] - (self.cpOffset[1] + self.ilChessC[0]) * (1 + self.cpKta * (ta - 25)) * (1 + self.cpKv * (vdd -3.3))

        for i in range(768):
            ilPattern = int(i / 32 - (i / 64) * 2)
            chessPattern = ilPattern ^ int(i - (i/2)*2)
            conversionPattern = ((i + 2) / 4 - (i + 3) / 4 + (i + 1) / 4 - i / 4) * (1 - 2 * ilPattern);

            if mode == 0:
                pattern = ilPattern
            else:
                pattern = chessPattern

    #         if pattern == frame[833]:
            irData = frame[i]
            if(irData > 32767):
                irData = irData - 65536


            irData = irData * gain

            irData = irData - self.offset[i]*(1 + self.kta[i]*(ta - 25))*(1 + self.kv[i]*(vdd - 3.3));
            if(mode !=  self.calibrationModeEE):
                irData = irData + self.ilChessC[2] * (2 * ilPattern - 1) - self.ilChessC[1] * conversionPattern; 


            irData = irData / emissivity;

            irData = irData - self.tgc * irDataCP[subPage];

            alphaCompensated = (self.alpha[i] - self.tgc * self.cpAlpha[subPage])*(1 + self.KsTa * (ta - 25));

            Sx = pow(alphaCompensated, 3) * (irData + alphaCompensated * taTr);
            Sx = np.sqrt(np.sqrt(Sx)) * KsTo[1]

            To = np.sqrt(np.sqrt(irData/(alphaCompensated * (1 - KsTo[1] * 273.15) + Sx) + taTr)) - 273.15;

            if(To < ct[1]):

                range_ = 0;

            elif(To < ct[2]):   

                range_ = 1;            

            elif(To < ct[3]):

                range_ = 2;            

            else:

                range_ = 3;            


            To = np.sqrt(np.sqrt(irData / (alphaCompensated * alphaCorrR[range_] * (1 + KsTo[range_] * (To - ct[range_]))) + taTr)) - 273.15;

            result[i] = To;

        return result
    

    def GetVideoStream(self, fps=8):

        refresh_rate = self.GetRefreshRate()

        if fps == 1:
            if refresh_rate != fps:
                self.SetRefreshRate(0b001);
        elif fps == 2:
            if refresh_rate != fps:
                self.SetRefreshRate(0b010);
        elif fps == 4:
            if refresh_rate != fps:
                self.SetRefreshRate(0b011);
        elif fps == 8:
            if refresh_rate != fps:
                self.SetRefreshRate(0b100);
        elif fps == 16:
            if refresh_rate != fps:
                self.SetRefreshRate(0b101);
        elif fps == 32:
            if refresh_rate != fps:
                self.SetRefreshRate(0b110);
        elif fps == 64:
            if refresh_rate != fps:
                self.SetRefreshRate(0b111);
        else:
            print('Incorrect fps setting')
            return 0

        while(True):
            frame = self.GetFrameData()

            image = np.zeros((24,32), np.float32)
            for y in range(24):
                for x in range(32): 
                    val1 = frame[0][32 * (23-y) + x]
                    val2 = frame[1][32 * (23-y) + x]
                    val3 = (val1+val2)/2
                    image[y][x] = val3
            imagef = cv2.GaussianBlur(image, (5,5), 0)
    #         ret,imagef = cv2.threshold(imagef, 20, 300, cv2.THRESH_TOZERO)
            imagef = cv2.resize(imagef, None, fx=20, fy=20, interpolation=cv2.INTER_LINEAR)
    #         image = cv2.normalize(image, 0, 1, cv2.NORM_MINMAX)
            imagef = (imagef/imagef.max())*255
    #         image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) 
            imagef = imagef.astype(np.uint8)
    #         ret,imagef = cv2.threshold(imagef,140,255,cv2.THRESH_TOZERO)
            imagef = cv2.applyColorMap(imagef, cv2.COLORMAP_JET)    
    #         print(image_color.shape)
            cv2.imshow('MLX90640 Thermal Imagery', imagef)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

        return image, imagef


    def GetFrameData(self):

        frame = [0,0]
        t_start = time.time()
        sub_cnt = 0

        ctrl_reg = self.getRegf(0x800D) 
        self.setRegf(0x8000, ctrl_reg ^ (1 << 3)) #set new data flag to 0
        new_data_flag = 0
        while(sub_cnt < 2):

            stat_reg = self.getRegf(0x8000) # status register
            ctrl_reg = self.getRegf(0x800D)
            new_data_flag = (stat_reg & 0x0008) >> 3 # flag for new data in RAM inside status register
            subpage = (stat_reg & 0x0001) # current subpage
            if new_data_flag == 1 and subpage == sub_cnt: # if flag is 1, there new data ready to read in RAM
    #             print(subpage, new_data_flag)

                self.setRegf(0x8000, ctrl_reg ^ (1 << 3)) #set new data flag to 0
                new_data_flag = 0


                frame[sub_cnt] = self.getRegs(0x0400,832) # read subpage from RAM

                frame[sub_cnt].append(ctrl_reg)
                frame[sub_cnt].append(stat_reg & 0x0001)            



                frame[sub_cnt] = self.InterpolateOutliers(frame[sub_cnt])


                frame[sub_cnt] = self.CalculateTo(frame[sub_cnt], 0.95)
                frame[sub_cnt] = self.BadPixelsCorrection(self.brokenPixels, frame[sub_cnt], 1 )
                frame[sub_cnt] = self.BadPixelsCorrection(self.outlierPixels, frame[sub_cnt], 1 )

                sub_cnt += 1


            t_end = time.time()
            t_elapsed = t_end - t_start

            if t_elapsed > 5:
                print('frameData timeout error waiting for dataReady')
                break


        return frame
        
