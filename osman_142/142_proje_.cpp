// NIHAD_231210088_ele142proje_revised.cpp
#include "canvas.cpp" // Bu dosyanın proje klasöründe olduğu varsayılmaktadır.
#include <cmath>
#include <iostream>

// PDF Kaynak [16, 50]: G=1 ve timestep (zaman adımı) tanımları
const double G = 1.0;
const double timestep = 0.05; // dt^2 << 1 kuralına uygun seçilmeli

// --- VECTOR SINIFI ---
// PDF Kaynak [52, 53, 54]: Veriler özel, operatörler aşırı yüklenmiş.
class Vector {
private:
    double x, y;

public:
    Vector(double x_val = 0, double y_val = 0) : x(x_val), y(y_val) {}

    // Erişimciler (Encapsulation gereği)
    double getX() const { return x; }
    double getY() const { return y; }

    // Operatör Aşırı Yükleme (Referans kullanarak performans artışı )
    Vector operator+(const Vector &v) const { return Vector(x + v.x, y + v.y); }
    Vector operator-(const Vector &v) const { return Vector(x - v.x, y - v.y); }
    Vector operator*(double k) const { return Vector(x * k, y * k); }
    Vector operator/(double k) const { return Vector(x / k, y / k); }

    double magnitude() const { return std::sqrt(x * x + y * y); }
    
    Vector unit() const {
        double m = magnitude();
        if (m == 0) return Vector(0, 0);
        return (*this) / m;
    }
};

// --- BODY (CİSİM) SINIFI ---
// PDF Kaynak [55, 56]: Kütle, hız, konum tutulmalı. Update fonksiyonu olmalı.
class Body {
protected: 
    // Türetilen 'Roket' sınıfı erişebilsin diye 'protected' yapıldı.
    // Dışarıdan erişim hala kapalıdır (Encapsulation).
    double mass;
    Vector pos;
    Vector vel;

public:
    Body(double m, const Vector &p, const Vector &v) 
        : mass(m), pos(p), vel(v) {}

    // Sanal Yıkıcı (Virtual Destructor) - Kalıtım kullanılan sınıflarda bellek sızıntısını önlemek için şarttır.
    virtual ~Body() {}

    // PDF Kaynak [56]: Bileşke kuvveti alıp hız ve konumu güncelleyen fonksiyon.
    // Taylor serisi yaklaşımı (Eq.3 ve Eq.4)
    virtual void update(const Vector &force) {
        if (mass <= 0) return; 
        Vector accel = force / mass;     // a = F/m (Eq.2)
        vel = vel + accel * timestep;    // v(t+dt) (Eq.3)
        pos = pos + vel * timestep;      // x(t+dt) (Eq.4)
    }

    // Erişimciler
    Vector getPos() const { return pos; }
    double getMass() const { return mass; }
    Vector getMomentum() const { return vel * mass; }
};

// --- ROKET SINIFI ---
// PDF Kaynak [35, 57, 59]: Body'den türetilmeli, ek itki ve kütle değişimi eklenmeli.
class Roket : public Body {
private:
    Vector v_exhaust;   // Egzoz gazı hızı (v_p)
    double w_exhaust;   // Birim zamanda atılan kütle (w_p)

public:
    Roket(double m, const Vector &p, const Vector &v, const Vector &ve, double we)
        : Body(m, p, v), v_exhaust(ve), w_exhaust(we) {}

    // PDF Kaynak [58, 59]: Temel update fonksiyonu ezilmeli (override).
    void update(const Vector &force) override {
        // PDF Eq.5: F_R = -w_p * v_p
        // Bu kuvvet, kütle çekim kuvvetine (force parametresi) eklenir.
        Vector thrust = v_exhaust * (-w_exhaust); 
        Vector totalForce = force + thrust;

        // İvme hesabı (Newton 2. Yasa)
        // Roket kütlesi anlık olarak değiştiği için anlık kütle kullanılır.
        if (mass > 0) {
            Vector accel = totalForce / mass;
            vel = vel + accel * timestep;
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
};

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
    ~LinkedList() {
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

    Node* getHead() const { return head; }
};

// --- UNIVERSE (EVREN) SINIFI ---
// PDF Kaynak [70, 71, 72]: Etkileşimleri yöneten sınıf.
class Universe {
private:
    LinkedList bodies;

public:
    // Body nesnesini listeye ekler.
    void insertBody(Body* b) {
        bodies.insert(b);
    }

    // PDF Kaynak [71, 129]: Simülasyonu koşturan fonksiyon.
    void run(double duration) {
        int steps = static_cast<int>(duration / timestep);
        
        for (int s = 0; s < steps; ++s) {
            Node* i = bodies.getHead();
            
            // Her cisim için kuvveti hesapla ve güncelle
            while (i != nullptr) {
                Vector net_force(0, 0);
                Node* j = bodies.getHead();
                
                while (j != nullptr) {
                    if (i != j) { // Kendisi hariç diğerleri
                        Vector diff = j->body->getPos() - i->body->getPos();
                        double dist = diff.magnitude();
                        
                        // Çarpışma/Sıfıra bölünme hatası önlemi (Softening parameter)
                        if (dist > 1e-4) {
                            // PDF Eq.1: F = G * m1 * m2 / r^2
                            double f_mag = G * i->body->getMass() * j->body->getMass() / (dist * dist);
                            net_force = net_force + diff.unit() * f_mag;
                        }
                    }
                    j = j->next;
                }
                
                // Kuvvet hesaplandıktan sonra cismi güncelle
                // Not: Daha hassas simülasyonlarda (ör. Verlet) önce tüm kuvvetler hesaplanır, 
                // sonra güncellemeler yapılır. Ancak proje tanımı (Eq.3, 4) Euler yöntemine işaret ediyor.
                i->body->update(net_force);
                i = i->next;
            }
        }
    }

    // Çizim için belirli bir indexteki cismin pozisyonunu döndürür
    Vector getPositionOfIndex(int index) {
        Node* curr = bodies.getHead();
        int count = 1; // Projede indexler 1'den başlıyor gibi görünüyor (m1, m2...)
        while (curr != nullptr) {
            if (count == index) {
                return curr->body->getPos();
            }
            curr = curr->next;
            count++;
        }
        return Vector(0,0);
    }
    
    // PDF Kaynak [161]: Fiziksel gözlemler (Ağırlık merkezi)
    Vector getCenterOfMass() const {
        Vector mr_sum(0,0);
        double m_sum = 0;
        Node* curr = bodies.getHead();
        while(curr) {
            mr_sum = mr_sum + (curr->body->getPos() * curr->body->getMass());
            m_sum += curr->body->getMass();
            curr = curr->next;
        }
        if(m_sum > 0) return mr_sum / m_sum;
        return Vector(0,0);
    }
};

// --- ANA FONKSİYON ---
int main() {
    // PDF'te belirtilen Canvas kullanımı
    canvas graphic("Uc_Cisim_Simulasyonu");
    graphic.startDoc();
    graphic.drawFrame();

    Universe U;

    // Kullanıcıdan dinamik veri alma kısmı (Korundu)
    int n;
    std::cout << "Simulasyona kac cisim eklenecek? ";
    std::cin >> n;

    for (int i = 0; i < n; ++i) {
        int type;
        std::cout << (i+1) << ". Cisim Tipi (0: Normal, 1: Roket): ";
        std::cin >> type;

        double m, px, py, vx, vy;
        std::cout << "Kutle, Pozisyon(x y), Hiz(x y): ";
        std::cin >> m >> px >> py >> vx >> vy;

        Vector pos(px, py);
        Vector vel(vx, vy);

        if (type == 0) {
            // new Body ile oluşturulan nesne, LinkedList yıkıcısı tarafından silinecek.
            U.insertBody(new Body(m, pos, vel));
        } else {
            double vex, vey, rate;
            std::cout << "Roket Puskurtme Hizi(x y) ve Orani(w): ";
            std::cin >> vex >> vey >> rate;
            U.insertBody(new Roket(m, pos, vel, Vector(vex, vey), rate));
        }
    }

    // Simülasyon döngüsü
    // PDF'teki örnekte 2000 adım koşturulmuş
    for (int t = 0; t < 2000; t++) {
        // Her çizim öncesi sistemi biraz ilerlet
        U.run(timestep * 10); // Her karede 10 fizik adımı at

        // İlk 3 cismi çizdir (Eğer varsa)
        // Renkler PDF örneğine uygun: Kırmızı, Yeşil, Mavi
        Vector p1 = U.getPositionOfIndex(1);
        Vector p2 = U.getPositionOfIndex(2);
        Vector p3 = U.getPositionOfIndex(3);

        if (n >= 1) graphic.drawPoint(p1.getX(), p1.getY(), "red");
        if (n >= 2) graphic.drawPoint(p2.getX(), p2.getY(), "green");
        if (n >= 3) graphic.drawPoint(p3.getX(), p3.getY(), "blue");
    }

    // Sonuçların ekrana basılması
    Vector com = U.getCenterOfMass();
    std::cout << "\nSimulasyon Bitti.\n";
    std::cout << "Sistemin Agirlik Merkezi: (" << com.getX() << ", " << com.getY() << ")\n";

    graphic.finishDoc();
    return 0; // Program bitince Universe -> LinkedList -> Node -> Body destructors çalışır.
}