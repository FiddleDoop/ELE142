#include "canvas.cpp"
#include <cmath>
#include <iostream>
//#include "gravity.cpp"

const double G = 1.0;//global olarak G değeri 1 alınması için bu şekilde seçildi değer
const double timestep = 0.05; // dt^2 << 1 kuralına uygun seçildi 


class vector {

public:
    vector(double, double);
    vector();
    // Erişimciler (Encapsulation gereği)
    double getX() const;
    double getY() const;
    // Operatör Aşırı Yükleme (Referans kullanarak performans artışı )
    vector operator+(const vector &v) const;
    vector operator-(const vector &v) const;
    vector operator*(double k) const;
    vector operator/(double k) const;
    double magnitude() const;
    vector unit() const;
private:
double x, y;//x ve y koordinatlarımız var
};
vector::vector(double valx,double valy):x(valx),y(valy){}
vector::vector():x(0),y(0){}
//vector sınıfının fonksiyonlarının tanıtılması:
double vector::getX() const{
    return x;
}
double vector::getY() const{
    return y;
}
vector vector::operator+(const vector &v) const{
    return vector(x+v.x,y+v.y);
}
vector vector::operator-(const vector &v) const{
    return vector(x-v.x,y-v.y);
}
vector vector::operator*(double k) const{
    return vector(x*k,y*k);
}
vector vector::operator/(double k) const{
    return vector(x/k,y/k);
}

double vector::magnitude() const{
    return std::sqrt(x*x+y*y);
}
vector vector::unit() const{//vektörün normalize edilmesini sağlayan kısım
    double m = magnitude();
    if(m==0){
        return vector(0,0);
    }
    return (*this)/m;
}


// PDF Kaynak [55, 56]: Kütle, hız, konum tutulmalı. Update fonksiyonu olmalı.
class Body {
public:
    Body(double m, const vector &p, const vector &v);
    // Sanal Yıkıcı (Virtual Destructor) - Kalıtım kullanılan sınıflarda bellek sızıntısını önlemek için şarttır.
    virtual ~Body() {}//sanal olmasaydı türetilen sınıfın yıkıcı fonksiyonu çağırılamazdı ve türetilen sınıftan olan nesneler de silinmek istediği zaman sadece temel sınıf silinirdi

    // PDF Kaynak [56]: Bileşke kuvveti alıp hız ve konumu güncelleyen fonksiyon.
    // Taylor serisi yaklaşımı (Eq.3 ve Eq.4)
    virtual void update(const vector &force);
    // Erişimciler
    vector getPos() const;
    double getMass() const;
    vector getMomentum() const;
protected: 
    double mass;
    vector pos;
    vector vel;

};
Body::Body(double m, const vector &p, const vector &v): mass(m), pos(p), vel(v) {}
void Body::update(const vector &force){
    if(mass <= 0){
        return;//kod void olsa bile buradaki return kendinden sonra gelen satırların çalışmasını engeller
    }
    vector ivme = force/mass; // a = F/m
    vel = vel + ivme * timestep; // v(t+dt) 
    pos = pos + vel * timestep; // x(t+dt)
}
vector Body::getPos() const {
    return pos;
}
double Body::getMass() const { 
    return mass; 
}
vector Body::getMomentum() const {
    return (vel * mass); 
}


// --- roket SINIFI ---
// PDF Kaynak [35, 57, 59]: Body'den türetilmeli, ek itki ve kütle değişimi eklenmeli.
class roket : public Body {//Body'den türetilen roket sınıfı
    //direkt olarak body sınıfından vector özelliklerini aldı miras olarak    
public:
    roket(double m, const vector &p, const vector &v, const vector &ve, double we);
    // PDF Kaynak [58, 59]: Temel update fonksiyonu ezilmeli (override).
    void update(const vector &force);
private:
    vector v_exhaust;   // Egzoz gazı hızı (v_p)
    double w_exhaust;   // Birim zamanda atılan kütle (w_p)

};


roket::roket(double m,const vector &p, const vector &v,const vector &ve,double we)
:Body(m,p,v),v_exhaust(ve),w_exhaust(we){}

void roket::update(const vector &force){
        // PDF Eq.5: F_R = -w_p * v_p
        // Bu kuvvet, kütle çekim kuvvetine (force parametresi) eklenir.
        vector thrust = v_exhaust * (-w_exhaust); 
        vector totalForce = force + thrust;

        // İvme hesabı (Newton 2. Yasa)
        // roket kütlesi anlık olarak değiştiği için anlık kütle kullanılır.
        if (mass > 0) {
            vector ivme = totalForce / mass;
            vel = vel + ivme * timestep;
            pos = pos + vel * timestep;
        }

        // PDF Eq.6: Kütle değişimi m(t+dt) = m(t) - w_p * dt
        mass = mass - (w_exhaust * timestep);

        // PDF Kaynak [47]: Kütle sıfırın altına düşemez (yakıt bitti/boş ağırlık).
        // Basitlik adına minimum 1 birim kütle kaldığını varsayıyoruz.
        if (mass < 1.0) {
            mass = 1.0;
            w_exhaust = 0; // Yakıt bittiği için artık itki yok.
        }
    }





// --- BAĞLI LİSTE YAPISI ---
// PDF Kaynak [66, 67, 69]: std::vector yasak, kendi bağlı listemiz olmalı.
struct Node {
    Body* body;
    Node* next;
    
    Node(Body* b) : body(b), next(nullptr) {}
};

class LinkedList {
private:
    Node* head;

public:
    LinkedList() : head(nullptr) {}

    // PDF Kaynak [138]: Bellek sızıntısını önlemek için yıkıcı metod.
    ~LinkedList() {//silinirken boşta düğüm kalmasın diye sadece en baştaki düğüm kalacak şekilde kendinden sonra gelecek düğümlerin silinmesi
        Node* current = head;
        while (current != nullptr) {
            Node* nextNode = current->next;
            delete current->body; // Body nesnesini sil
            delete current;       // Düğümü sil
            current = nextNode;
        }
    }

    void insert(Body* b) {
        Node* newNode = new Node(b);
        newNode->next = head;
        head = newNode;
    }

    Node* getListFirst() const { return head; }
};

// --- UNIVERSE (EVREN) SINIFI ---
// PDF Kaynak [70, 71, 72]: Etkileşimleri yöneten sınıf.
class evren {
private:
    LinkedList bodies;

public:
    void insertBody(Body* b);
    void run(double duration);
    vector getposition(int index);
    vector getMassCenter() const;
};
void evren::insertBody(Body *b){
    bodies.insert(b);
}
void evren::run(double duration){
    int steps = static_cast<int>(duration / timestep);
        
        for (int s = 0; s < steps; ++s) {
             Node* i = bodies.getListFirst();
            
             // Her cisim için kuvvetin hesaplanması ve cisim durumlarının güncellenmesi
             while (i != nullptr) {
                 vector net_force(0, 0);
                 Node* j = bodies.getListFirst();
                
                 while (j != nullptr) {
                     if (i != j) { // Kendisi hariç diğerleri
                         vector diff = j->body->getPos() - i->body->getPos();
                         double dist = diff.magnitude();
                        
                         // Çarpışma/Sıfıra bölünme hatası önlenmesi için 0'a çok yakın değer verilmesi
                         if (dist > 1e-4) {
                             //dokümanda yer alan F = G * m1 * m2 / r^2 denkleminin uygulanması 
                             double f_mag = G * i->body->getMass() * j->body->getMass() / (dist * dist);
                             net_force = net_force + diff.unit() * f_mag;
                         }
                     }
                     j = j->next;
                 }
                
                 // Kuvvet hesaplandıktan sonra cisim güncellenir
                 i->body->update(net_force);
                 i = i->next;
             }
         }
}
vector evren::getposition(int index) {
        Node* curr = bodies.getListFirst();
        int count = 1; // Index'ler 1'den başlamakta olduğundan
        while (curr != nullptr) {
            if (count == index) {
                return curr->body->getPos();
            }
            curr = curr->next;
            count++;
        }
        return vector(0,0);
    }
vector evren::getMassCenter() const {
        vector mr_sum(0,0);
        double m_sum = 0;
        Node* curr = bodies.getListFirst();
        while(curr) {
            mr_sum = mr_sum + (curr->body->getPos() * curr->body->getMass());
            m_sum += curr->body->getMass();
            curr = curr->next;
        }
        if(m_sum > 0) return mr_sum / m_sum;
        return vector(0,0);
    }














int main() {
    //proje dokümanındaki gibi canvas kullanımı
    canvas graphic("Uc_Cisim_Simulasyonu");
    graphic.startDoc();
    graphic.drawFrame();

    evren U;

    // Kullanıcıdan dinamik veri alma kısmı
    int n;
    std::cout << "Simulasyona kac cisim eklenecek? ";
    std::cin >> n;

    for (int i = 0; i < n; ++i) {
        int type;
        std::cout << (i+1) << ". Cisim Tipi (0: Normal, 1: roket): ";
        std::cin >> type;

        double m, px, py, vx, vy;
        std::cout << "Kutle, Pozisyon(x y), Hiz(x y): ";
        std::cin >> m >> px >> py >> vx >> vy;

        vector pos(px, py);
        vector vel(vx, vy);

        if (type == 0) {
            // new Body ile oluşturulan nesne, LinkedList yıkıcısı tarafından silinecek.
            U.insertBody(new Body(m, pos, vel));
        } else {
            double vex, vey, rate;
            std::cout << "roket Puskurtme Hizi(x y) ve Orani(w): ";
            std::cin >> vex >> vey >> rate;
            U.insertBody(new roket(m, pos, vel, vector(vex, vey), rate));
        }
    }

    for (int t = 0; t < 2000; t++) {
        
        U.run(timestep * 10); // Her karede 10 fizik adımı at

        // İlk 3 cismi çizdir (Eğer varsa)
        // Renkler PDF örneğine uygun: Kırmızı, Yeşil, Mavi
        vector p1 = U.getposition(1);
        vector p2 = U.getposition(2);
        vector p3 = U.getposition(3);

        if (n >= 1) graphic.drawPoint(p1.getX(), p1.getY(), "red");
        if (n >= 2) graphic.drawPoint(p2.getX(), p2.getY(), "green");
        if (n >= 3) graphic.drawPoint(p3.getX(), p3.getY(), "blue");
    }

    // Sonuçların ekrana basılması
    vector com = U.getMassCenter();
    std::cout << "\nSimulasyon Bitti.\n";
    std::cout << "Sistemin Agirlik Merkezi: (" << com.getX() << ", " << com.getY() << ")\n";

    graphic.finishDoc();
    return 0; 
}