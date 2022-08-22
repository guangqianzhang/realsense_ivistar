## 调试记录

### 摄像头

### 红绿灯检测

1. 未启动状态下检测到绿色（lighttype=3）计数像素点最大49，可以考虑过滤较小的计数点，但可能影响到远距离时的检测。trafficlight.hpp line=134
2. 根据场景设计颜色过滤器。 trafficlight.hpp line=58

   ```
   Scalar RedLow=Scalar (0,43,46);
   Scalar RedUp=Scalar (10,255,255);
   Scalar RedLow2=Scalar (156,43,46);
   Scalar RedUp2=Scalar (180,255,255);
   Scalar YellowLow=Scalar (26,43,46);
   Scalar YellowUp=Scalar (34,255,255);
   Scalar GreenLow=Scalar (35,43,46);
   Scalar GreenUp=Scalar (77,255,255);
   ```

   ```
       inRange(hsv, RedLow, RedUp, dstR1);
       inRange(hsv, RedLow2, RedUp2, dstR2);
       Mat dstR =dstR1|dstR2;
       inRange(hsv,YellowLow,YellowUp,dstY);
       inRange(hsv,GreenLow,GreenUp,dstG);
   ```

### 红旗启动
