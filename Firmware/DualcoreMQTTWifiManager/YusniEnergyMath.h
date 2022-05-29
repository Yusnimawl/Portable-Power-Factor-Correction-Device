/*
 * /fungsi2 matematika
 * derajatkeradian(int sudut)
 * radiankederajat(double radian)
 * carisudutfasa(double selisih, double periodefrek)
 * cariVA(double V, double I)
 * cariWATT(double VA, double cosphi)
 * caricosphidariPdanS(double P, double S)
 * cariCuF(int V, double Q, double F)
 * cariVAR(double VA, double sudutdalamradian)
 * cariLH(int V, double Q, double F)
 * cariCuF(int V, double Q, double F)
 */

//program konversi derajat ke radian(input: sudut 0-360)
double derajatkeradian(int sudut){
double hasil=(sudut)*PI/180;
return hasil;}

//program konversi radian ke derajat(input: radian)
int radiankederajat(double radian){
int hasil = (radian)*180/PI;
return hasil;
}

//program mencari sudut pergeseran dari zero crossing(input: ton dan periode(50hz=20ms))
int carisudutfasa(double selisih, double periodefrek){
int hasil = (selisih/periodefrek)*360;
return hasil;
}


//program mencari daya reaktif Q/VAR(input: daya nyata(va) dan sudut(dalam radian))
int cariVAR(double VA, double sudutdalamradian){
int hasil = VA*sin(sudutdalamradian);
return hasil;
}

//program mencari daya semu S/VA(input: tegangan dan arus)
int cariVA(double V, double I){
int hasil = V*I;
return hasil;
}

//program mencari daya nyata P/Watt(input: tegangan, arus, dan faktor daya)
int cariWATT(double VA, double cosphi){
int hasil = VA*cosphi;
return hasil;
}

//program mencari cosphi dari perbandingan P dan S(input: P(daya nyata) dan S(daya semu))
int caricosphidariPdanS(double P, double S){
int hasil = P/S;
return hasil;
}

//program mencari nilai Kapasitor dalam mikrofarad(input: V(tegangan), Q(VAR) dan F(frekuensi))
int cariCuF(int V, double Q, double F){
double XC = (V*V)/(Q);
int C = (1/(2*PI*F*XC))*1000000;
return C;
}

//program mencari nilai Induktor(input: V(tegangan), Q(VAR) dan F(frekuensi))
int cariLH(int V, double Q, double F){
double XL = (V*V)/(Q);
int L = 2*PI*F*XL;
return L;
}
