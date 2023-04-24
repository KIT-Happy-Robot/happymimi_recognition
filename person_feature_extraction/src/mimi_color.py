#!/usr/bin/env python3
#-*- coding: utf-8 -*

class DetectColor():  
    def detectColor(self,hsv_bit_image):
        B = 'black'
        W = 'white'
        G = 'gray'
        self.h,s,v = hsv_bit_image
        self.ps = 100 / 255 * s
        self.pv = 100 / 255 * v
        print('ps:{},pv:{}'.format(self.ps,self.pv))

        if self.ps <  5:
            color = self.if3Color(15,B,60,G,W) #Black < 15 < Gray < 60 <= White
        elif self.ps < 10:
            color = self.if3Color(20,B,50,G,W) #Black < 20 < Gray < 50 <= White
        elif self.ps < 20:
            color = self.if3Color(20,B,30,G,self.putColor()) #Black < 20 <= Gray < 30 <= Color
        elif self.ps < 40:
            color = self.if2Color(15,B,self.putColor()) #Black < 15 <= Color
        elif self.ps < 60:
            color = self.if2Color(10,B,self.putColor()) #Black < 10 <= Color
        else:
            color = self.putColor()

        return color    
     
    def putColor(self):
        if self.h < 5 or 170 < self.h:
            if self.pv < 40:    color = 'brown'
            else:   color = 'red'
        elif self.h < 20:
            if self.pv < 45:    color = 'brown'
            else:   color = 'orange'
        elif self.h < 30:
            if self.pv < 50:    color = 'brown'
            else:   color = 'yellow'
        elif self.h < 40:
            color = 'yellow green'
        elif self.h < 50:
            color = 'green'
        elif self.h < 80:
            if self.pv < 30:    color = 'green'
            else:   color = 'blue green'
        elif self.h < 100:
            if self.pv < 20:    color = 'green'
            else:   color = 'light blue'
        elif self.h < 130:  
            color = 'blue'
        elif self.h < 140:  
            color = 'purple'
        else:   
            color = 'pink'

        return color
    

    def if2Color(self,pv_range1,color1,color2):
        if self.pv < pv_range1:
            color = color1
        else:
            color = color2
        return color

    def if3Color(self,pv_range1,color1,pv_range2,color2,color3):
        if self.pv  < pv_range1:
            color = color1
        elif self.pv < pv_range2:
            color = color2
        else:
            color = color3
        return color
    
'''
#example

dcolor = DetectColor()
color = dcolor.detectColor(image[y,x,:])
print(color)
'''