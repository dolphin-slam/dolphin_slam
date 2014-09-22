#ifndef COLOR_H
#define COLOR_H

namespace dolphin_slam
{

class Color {
    float r;
    float g;
    float b;

public:
    Color (float gray = 0){
        setColor(gray);
    }

    void setColor(float gray){
       r = gray;
       if(gray > 0.5){
           g = -2*gray+2;
       }else{
           g = 2*gray;
       }
       b = 1-gray;
    }

    float getR(){
        return r;
    }
    float getG(){
        return g;
    }
    float getB(){
        return b;
    }

};

}   // namespace

#endif // COLOR_H
