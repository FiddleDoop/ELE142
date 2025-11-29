#include "canvas.cpp"
#include <cmath>
#include <iostream>

const double G=1.0;     //yercekimi sabiti
const double timestep=0.01;     // timestep (bu değeri değiştirecegiz duruma göre)

class vector{
    private:
    double x,y;

    public:
    void setX(double newX){ x=newX; }
    void setY(double newY){ y=newY; }
    double getX() const { return x; }
    double getY() const { return y; }
    void print() const { std::cout<<"("<<x<<","<<y<<")"<<std::endl; }   // yardımcı fonksiyonlarımız
    vector(double xdummy=0, double ydummy=0):x(xdummy),y(ydummy) {} // constructor
    vector operator+(const vector &v_right) const { return vector(x + v_right.x, y + v_right.y); } // toplama operatörünü overload ettik
    vector operator-(const vector &v_right) const { return vector(x - v_right.x, y - v_right.y); } // çıkarma operatörünü overload ettik
    vector operator*(double s) const { return vector(x * s, y * s); } // skaler çarpma operatörünü overload ettik
    vector operator/(double b) const { return vector(x / b, y / b); } // skaler bölme operatörünü overload ettik

    double absouluteValue() const { return sqrt(x * x + y * y); } // vektörün büyüklüğünü hesaplayan fonksiyon |vector|
    vector unit_vector() const {   // birim vektör hesaplayan(dönen) fonksiyon |vector|=1 için vektör döndürüyor
        double mutlakdeger = absouluteValue();
        if (mutlakdeger == 0)
            return vector(0, 0); // burada bölme hatasından kaçınıyoruz çok önemli
        else
            return (*this) / mutlakdeger;
    }

};

class Body {
    protected:
    double mass;
    vector position, velocity;
    public:
    Body(double m,const vector& pos,const vector& vel):mass(m),position(pos),velocity(vel) {}
    virtual void update(const vector& force) {
        vector acceleration = force / mass; // a = Force/mass
        velocity = velocity + acceleration * timestep; // v = v0 + a*t
        position = position + velocity * timestep; // x = x0 + v*t
    }   
    virtual vector getposition() const { return position; }
    virtual double getmass() const { return mass; }
    virtual ~Body() {}    
    virtual vector getmomentum() const { return velocity * mass; }
};

class Rocket:public Body { // Roket sınıfı Body sınıfından türetiliyor
    protected:
    vector yavaslama_velocity;
    double yavaslama_orani;
    public:
    Rocket(double m, const vector& pos,const  vector& vel, const vector& ev, double rate)
        : Body(m,pos,vel),yavaslama_velocity(ev),yavaslama_orani(rate) {}

    virtual void update(const vector& force) {
        vector thrust = yavaslama_velocity * yavaslama_orani; // roketin itme kuvveti
        vector Totalforce = force + thrust; // toplam kuvvet = yerçekimi kuvveti + itme kuvveti force = force + thrust aslında ama const olduğu için yeni bir değişkene atıyoruz
        Body::update(Totalforce); // temel sınıfın update fonksiyonunu çağırıyoruz force ile
        mass -= yavaslama_orani * timestep; // kütleyi azaltıyoruz cünkü yakıt harcanıyor
        if (mass < 1.0) mass = 1.0; // kütle çok küçülürse 1.0 yapalım çünkü negatif kütle olmaz hatadan kaçınalım

    }
};
