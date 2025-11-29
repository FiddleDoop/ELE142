/*
 *   TOBB ETU Elektrik ve Elektronik Bölümü
 *   ELE142 Bilgisayar Programlama II
 *   Grafik Çizim Kütüphanesi
 *   o.arik@edu.etu.tr
 */

#include <fstream>
#include <cmath>


#define Width 1000
#define Height 1000
#define leftX 0
#define centerX 500
#define rightX 1000
#define topY 0
#define centerY 500
#define bottomY 1000 

class canvas
{
public:
    canvas(std::string filename, double scaleX=1, double scaleY=1, int offsetX=centerX, int offstY=centerY);
    void openFile(bool newfile=false);
    void closeFile();
    void startDoc();
    void finishDoc();
    void drawFrame();
    void drawPolyline(double px[], double py[], int size, std::string color="black");
    void drawPolygon(double px[], double py[], int size, std::string color="black");
    void drawPoints(double px[], double py[], int size, std::string color="black", double radius=1);
    void drawLine(double px1, double py1, double px2, double py2, std::string color="black");
    void drawRectangle(double px, double py, double width, double height, std::string color="black");
    void drawPoint(double px, double py, std::string color="black", double radius=1);
    void drawArray(double array[], int size, std::string color="black", bool expand=true);
    void drawText(std::string text, double px, double py, std::string color="grey", int fontsize=14);
    double mapX(double x) const;
    double mapY(double y) const;

private:
    std::string filename, title;
    std::ofstream outputFile;
    int offsetX;
    int offsetY;
    double scaleX;
    double scaleY;
};

canvas::canvas(std::string name, double scaleX, double scaleY, int offsetX, int offsetY) : 
filename(name), offsetX(offsetX), offsetY(offsetY), scaleX(scaleX), scaleY(scaleY)
{  
    size_t pos = filename.find(".html");
    size_t length = filename.length();
    if(pos == std::string::npos)
        filename.append(".html");
    else if (pos+5 != length)
        filename.append(".html");
    
    title = filename;    
    pos = title.find(".html");
    title.erase(pos, 5);
}

void canvas::openFile(bool newfile)
{
    if(newfile)
        outputFile.open(filename);
    else
        outputFile.open(filename,std::ofstream::app);
}

void canvas::closeFile()
{
    outputFile.close();
}


double canvas::mapX(double x) const
{
    return x*scaleX+offsetX;
}

double canvas::mapY(double y) const
{
    return -y*scaleY+offsetY;
}

void canvas::drawPolyline(double px[], double py[], int size, std::string color)
{
    openFile();
    outputFile << "ctx.beginPath();\n";
    outputFile << "ctx.moveTo("<< mapX(px[0]) << "," << mapY(py[0]) << ");\n";
    for(int i=1; i<size; i++)
    {
        outputFile << "ctx.lineTo(" << mapX(px[i]) << "," << mapY(py[i]) << ");\n";

    }
    outputFile << "ctx.strokeStyle = \"" << color << "\";\n";
    outputFile << "ctx.stroke();\n";
    closeFile();  
}

void canvas::drawPolygon(double px[], double py[], int size, std::string color)
{
    openFile();
    outputFile << "ctx.beginPath();\n";
    outputFile << "ctx.moveTo("<< mapX(px[0]) << "," << mapY(py[0]) << ");\n";
    for(int i=1; i<size; i++)
    {
        outputFile << "ctx.lineTo(" << mapX(px[i]) << "," << mapY(py[i]) << ");\n";

    }
    outputFile << "ctx.closePath();\n";
    outputFile << "ctx.strokeStyle = \"" << color << "\";\n";
    outputFile << "ctx.stroke();\n";
    closeFile();  
}


void canvas::drawPoints(double px[], double py[], int size, std::string color, double radius)
{
    openFile();
    outputFile << "ctx.beginPath();\n";
    for(int i=0; i<size; i++)
    {
        outputFile << "ctx.moveTo("<< mapX(px[i])+radius << "," << mapY(py[i]) << ");\n";
        outputFile << "ctx.arc(" << mapX(px[i]) << "," << mapY(py[i]) << "," << radius << ", 0, 2 * Math.PI);\n";
    }
    outputFile << "ctx.strokeStyle = \"" << color << "\";\n";
    outputFile << "ctx.stroke();\n";
    closeFile();  
}

void canvas::drawLine(double px1, double py1, double px2, double py2, std::string color)
{
    openFile();
    outputFile << "ctx.beginPath();\n";
    outputFile << "ctx.moveTo(" << mapX(px1) << "," << mapY(py1) << ");\n";
    outputFile << "ctx.lineTo(" << mapX(px2) << "," << mapY(py2) << ");\n";
    outputFile << "ctx.strokeStyle = \"" << color << "\";\n";
    outputFile << "ctx.stroke();\n";
    closeFile();
}

void canvas::drawRectangle(double px, double py, double width, double height, std::string color)
{
    openFile();
    outputFile << "ctx.strokeStyle = \"" << color << "\";\n";
    outputFile << "ctx.strokeRect(" << mapX(px) << "," << mapY(py) << "," << width*scaleX << "," << height*scaleY << ");\n";;
    closeFile();
}

void canvas::drawPoint(double px, double py, std::string color, double radius)
{
    openFile();
    outputFile << "ctx.beginPath();\n";
    outputFile << "ctx.moveTo("<< mapX(px)+radius << "," << mapY(py) << ");\n";
    outputFile << "ctx.arc(" << mapX(px) << "," << mapY(py) << "," << radius << ", 0, 2 * Math.PI);\n";
    outputFile << "ctx.strokeStyle = \"" << color << "\";\n";
    outputFile << "ctx.stroke();\n";
    closeFile(); 
}

void canvas::drawArray(double array[], int size, std::string color, bool expand)
{
    if(expand)
    {
        offsetX = leftX;
        scaleX = Width/(double)size;
    }
    openFile();
    outputFile << "ctx.beginPath();\n";
    outputFile << "ctx.moveTo("<< mapX(0) << "," << mapY(array[0]) << ");\n";
    for(int i=1; i<size; i++)
    {
        outputFile << "ctx.lineTo(" << mapX(i) << "," << mapY(array[i]) << ");\n";
    }
    outputFile << "ctx.strokeStyle = \"" << color << "\";\n";
    outputFile << "ctx.stroke();\n";
    closeFile(); 
}

void canvas::drawText(std::string text, double px, double py, std::string color, int fontsize)
{
    openFile();
    outputFile << "ctx.font = \"" << fontsize << "px Consolas\";\n";
    outputFile << "ctx.fillStyle = \"" << color << "\";\n";
    outputFile << "ctx.textAlign = \"center\";\n";
    outputFile << "ctx.fillText(\"" << text << "\"," << mapX(px) << "," << mapY(py) << ");\n";
    closeFile();
}


void canvas::drawFrame()
{
    openFile();
    outputFile << "ctx.beginPath();\n";
    outputFile << "ctx.font = \"14px Consolas\";\n";
    outputFile << "ctx.fillStyle = \"grey\";\n";
    outputFile << "ctx.moveTo(0,0);\n";
    outputFile << "ctx.lineTo(" << Width << ",0);\n";
    outputFile << "ctx.lineTo(" << Width << "," << Height << ");\n";
    outputFile << "ctx.lineTo(0," << Height << ");\n";
    outputFile << "ctx.lineTo(0,0);\n";

    outputFile << "ctx.textAlign = \"center\";\n";

    double precX = pow(10,round(log10(scaleX)));
    double precY = pow(10,round(log10(scaleY)));

    for(int x=100; x<Width; x+=100) // up
    {
        outputFile << "ctx.moveTo(" << x << ",0);\n";
        outputFile << "ctx.lineTo(" << x << ",10);\n";
        outputFile << "ctx.fillText(\"" << (int)(round(precX*(x-offsetX)/scaleX))/precX << "\"," << x << ",25);\n";
    }

    for(int x=100; x<Width; x+=100) // down
    {
        outputFile << "ctx.moveTo(" << x << "," << Height-10 << ");\n";
        outputFile << "ctx.lineTo(" << x << "," << Height << ");\n";
        outputFile << "ctx.fillText(\"" << (int)(round(precX*(x-offsetX)/scaleX))/precX << "\"," << x << "," << Height-15 << ");\n";
    }

    outputFile << "ctx.textAlign = \"left\";\n";

    for(int y=100; y<Height; y+=100) // left
    {
        outputFile << "ctx.moveTo(0," << y << ");\n";
        outputFile << "ctx.lineTo(10," << y << ");\n";
        outputFile << "ctx.fillText(\"" << (int)(round(-precY*(y-offsetY)/scaleY))/precY << "\",15," << y+5 << ");\n";
    }

    outputFile << "ctx.textAlign = \"right\";\n";

    for(int y=100; y<Height; y+=100) // right
    {
        outputFile << "ctx.moveTo(" << Width-10 << "," << y << ");\n";
        outputFile << "ctx.lineTo(" << Width << "," << y << ");\n";
        outputFile << "ctx.fillText(\"" << (int)(round(-precY*(y-offsetY)/scaleY))/precY << "\"," << Width-15 << "," << y+5 << ");\n";
    }

    outputFile << "ctx.strokeStyle = \"grey\";\n";
    outputFile << "ctx.stroke();\n";
    closeFile();
}


void canvas::startDoc()
{
    openFile(true);

    outputFile << "<!--\n"
               <<"TOBB ETU Elektrik ve Elektronik Bölümü\n"
               << "ELE142 Bilgisayar Programlama II\n"
               << "Grafik Çizim Kütüphanesi ile oluşturulmuştur.\n"
               << "o.arik@edu.etu.tr\n"
               << "-->\n";

    outputFile << "<!DOCTYPE html>\n";
    outputFile << "<html>\n";
    outputFile << "<body style=\"width: " << Width << "px;\">\n";

    outputFile << "<p style=\"text-align: center;\"><b>TOBB ETÜ ELEKTRİK VE ELEKTRONİK BÖLÜMÜ</b></p>\n";
    outputFile << "<h1 style=\"text-align: center;\">ELE142 Bilgisayar Programlama II</h1>\n";
    outputFile << "<h2 style=\"text-align: center;\">" << title <<"</h2>\n";

    outputFile << "<canvas id=\"myCanvas\" width=\"" << Width <<"px\" height=\"" << Height << "px\" style=\"border:1px solid #000000;\">\n";
    outputFile << "Sorry, your browser does not support canvas.\n";
    outputFile << "</canvas>\n";

    outputFile << "<script>\n";
    // Create a canvas
    outputFile << "const canvas = document.getElementById(\"myCanvas\");\n";
    outputFile << "const ctx = canvas.getContext(\"2d\");\n";

    closeFile();
}


void canvas::finishDoc()
{
    openFile();

    outputFile <<"</script>\n";
    outputFile <<"</body>\n";
    outputFile <<"</html>\n";
    
    closeFile();

}