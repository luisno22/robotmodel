#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <math.h>
#include <Yml.hpp>
#include <cmath>
#include <vector>
//#include <libs/parameters_yml.hpp>

#define delta_giro		5
#define giro_max		  10
#define PI            3.14159265359
#define number_of_models 1

int tamano;

using namespace std;

  //Método de eliminación de Gauss: eliminación hacia adelante.
  void eliminacionGauss(double **A, double B[], int n)
  {
    double inv;
    for (int k = 0; k < n; k++)
    {
      for (int i = k + 1; i < n; i++)
      {
        inv = A[i][k] / A[k][k];
        for (int j = k; j < n; j++)
        {
          A[i][j] = A[i][j] - inv * A[k][j];
        }
        B[i] = B[i] - inv * B[k];
      }
    }
  }

  //Método de eliminación de Gauss: sustitución inversa.
  void sustitucionAtras(double **A, double B[], int n, double C[])
  {
    double suma;
    C[n - 1] = B[n - 1] / A[n - 1][n - 1];
    for (int i = n - 2; i >= 0; i--)
    {
      suma = 0;
      for (int j = i + 1; j < n; j++)
      {
        suma = suma + A[i][j] * C[j];
      }
      C[i] = (B[i] - suma) / A[i][i];
    }
  }


  //Regresión polinomial: recibe los datos de x y y, el grado del
  //polinomio resultante y el número de datos.
  vector<double> regresionPolinomial(double x[], double y[], int n, int m)
  {
    double sum_x = 0, sum_xy = 0;
    tamano = m + 1;
    double *solucion = new double[tamano];
    double **ecuaciones;

    ecuaciones = new double *[tamano];
    // Inicialización del arreglo bidimensional.
    for (int i = 0; i < tamano; i++)
    {
      ecuaciones[i] = new double[tamano];
    }

    // Cálculo de las sumatorias y armado del sistema.
    for (int i = 0; i < tamano; i++)
    {
      sum_xy = 0;

      for (int j = 0; j < n; j++)
        sum_xy += pow(x[j], i) * y[j];
      solucion[i] = sum_xy;

      for (int j = 0; j < tamano; j++)
      {
        sum_x = 0;
        if (i == 0 && j == 0)
          ecuaciones[i][j] = n;
        else
        {
          for (int h = 0; h < n; h++)
            sum_x += pow(x[h], (j + i));
          ecuaciones[i][j] = sum_x;
        }
      }
    }

    // Resolucion de sistemas de ecuaciones.
    eliminacionGauss(ecuaciones, solucion, tamano);

    double *x_vector = new double[tamano];

    sustitucionAtras(ecuaciones, solucion, tamano, x_vector);

    // Construcción de la ecuación final.
    // cout << "La ecuacion es: ";
    // for (int i = 0; i < tamano; i++)
    // {
    //   cout << x_vector[i];
    //   if (i != 0)
    //   {
    //     cout << "x^" << i;
    //   }
    //   if (i != tamano - 1)
    //   {
    //     cout << " + ";
    //   }
    // }
    // cout << endl;

    // Cálculo de los errores
    double *e = new double[n];
    for (int i = 0; i < n; i++)
    {
      double y_calculada = 0;
      for (int j = tamano - 1; j >= 1; j--)
        y_calculada += x_vector[j] * (pow(x[i], j));
      y_calculada += x_vector[0];
      e[i] = pow(y[i] - y_calculada, 2);
    }

    double sum_y = solucion[0];

    double sr = 0;
    double st = 0;
    for (int i = 0; i < n; i++)
    {
      sr += e[i];
      st += pow(y[i] - (sum_y / n), 2);
    }

    double err = sqrt(sr / (n - tamano));

    double r2 = (st - sr) / st;
    double r = sqrt(r2);
    // Desplegado de errores.
    // cout << "Error estandar de la estimacion: " << err << endl;
    // cout << "Coeficiente de determinacion: " << r2 << endl;
    // cout << "Coeficiente de correlacion: " << r << endl;
    // cout << endl;
    vector<double> output;
    for(int i=0;i<tamano;i++){
      output.push_back(x_vector[i]);
    }
    return output;
  }

  void modelRL(float *, float *, float);
  void write_yml_groups(std::ofstream &, int, float, float);
  void tf_position(float, float, float, float, float, float, float *);

  int main(int argc, char *argv[])
  {
    ifstream filex("vectorx.txt");
    ifstream filey("vectory.txt");
    if (!filex.is_open() || !filey.is_open())
    {
      cout << "Error al abrir ejemplo.dat\n";
      exit(EXIT_FAILURE);
    }

    double valorx, valory;
    vector<vector<double>> coefs;
    vector<double> subx;
    vector<double> suby;
    while((filex >> valorx)&&(filey >> valory)){  //Recorre los archivos de texto
      if(valorx==0 && valory==0 && subx.size()>2){ // Si pasa por el punto (0,0) y además hay al menos 2 valores no nulos en ese intervalo, almacena el intervalo completo
        //Incluye el punto (0,0) como punto inicial del intervalo
        subx.insert(subx.begin(),0);
        suby.insert(suby.begin(),0);
        // convierte ambos vectores en arrays
        double* arco_x_p = new double[subx.size()];
        std::copy(subx.begin(), subx.end(), arco_x_p);
        double* arco_y_p = new double[suby.size()];
        std::copy(suby.begin(), suby.end(), arco_y_p);
        // Realiza la regresion poliniomial para ese intervalo
        vector<double> coef = regresionPolinomial(arco_x_p,arco_y_p,subx.size(),2);
        // Añade los coeficientes de ese intervalo al vector de coeficientes
        coefs.push_back(coef);
        // Limpia los vectores que almacenan el intervalo
        subx.clear();
        suby.clear();
      }

      // Almacena los valores del intervalo
      subx.push_back(valorx);
      suby.push_back(valory);
    }

    for(int i=0;i<coefs.size();i++){
      cout<< "Coeficientes de la curva que define la trayectoria numero: "<<i<<" --> ";
      for(int j=0;j<coefs[i].size();j++){
        cout<<coefs[i][j]<<"*x^"<<j<<" + ";
      }
      cout<<endl;
    }
    cout<<endl;

    return 0;
  }

  void write_yml_groups(std::ofstream & file, int ciclo, float velocidad, float giro)
  {
    file << "ciclo_" + std::to_string(ciclo) + ": \n giro: " + std::to_string(giro) + " \n velocidad: " + std::to_string(velocidad) + "\n vectorx:\n  - \n vectory:\n  - \n vectortheta:\n  - \n";
  }

  /// -----  EN DESARROLLO ----- ///
  /// @brief Funcion que modifica la posicion actual por la siguiente posicion una vez aplicado el control necesario durante el tiempo especificado
  /// @param pose array de 3 valores almacenando x,y,theta respectivamente, se modifica durante la funcion
  /// @param control array de 2 valores almacenando velocidad y giro respectivamente
  /// @param t tiempo durante el que se aplica ese control
  void modelRL(float *pose, float *control, float t)
  {
    float x_out;
    float y_out;
    float theta_out;
    int n_trajectory; //variable que indica el numero de tramo en el que se encuentra
    std::ifstream myfilex; // Se abre el archivo que almacena el vector de posiciones x
    myfilex.open("vector_x.txt"); 
    vector<float> x_vector;
    vector<int> index_per_trajectory; // vector de enteros que almacena los indices del inicio de cada ciclo en el vector de posiciones
    string line;
    int i=0;

    // Recorre el archivo de texto, almacena los valores en un vector y guarda los indices de las posiciones iguales a 0
    while (getline(myfilex, line)) {
      x_vector.push_back(stof(line));
      if(x_vector.back() == 0.0){
        index_per_trajectory.push_back(i);
      }
      i++;
    }
    
    // Numero de trayectoria en la que se encuentra el valor de control indicado
    n_trajectory = int(floor(((giro_max+giro_max)/delta_giro) - ((control[1]+giro_max)/delta_giro)));


    //Variables auxiliares hasta que decida como almacenar el modelo
    float x_array[100];
    float y_array[100];
    float theta_array[100];
    float t_array[100];


    // Indices en los que empieza y termina el tramo en el que se encuentra el valor de control indicado
    int index_inicio_tramo1;
    int index_final_tramo1;
    int index_final_tramo2;
    if(control[0]>0){
      index_inicio_tramo1 = index_per_trajectory[n_trajectory*2];
      index_final_tramo1 = index_per_trajectory[n_trajectory*2 + 1];
      index_final_tramo2 = index_per_trajectory[n_trajectory*2 + 2];
    }else if(control[0]<0){
      index_inicio_tramo1 = index_per_trajectory[n_trajectory*2 + 1];
      index_final_tramo1 = index_per_trajectory[n_trajectory*2 + 2];
      index_final_tramo1 = index_per_trajectory[n_trajectory*2 + 3];
    }
    i=index_inicio_tramo1;
    do{
        i++;
    }while(t<(t_array[index_inicio_tramo1+i]-t_array[index_inicio_tramo1]));
    float t_tf = t + t_array[index_inicio_tramo1]; // Tiempo real total sumando el punto de inicio de esa trayectoria
    float percent_time = (t_tf-t_array[i-1])/(t_array[i]-t_array[i-1]); // Porcentaje del total del segmento de tiempo en el que se encuentra
    float dx_segment = (x_array[i]-x_array[i-1])*percent_time; // Incremento de x en el segmento
    float dy_segment = (y_array[i]-y_array[i-1])*percent_time; // Incremento de y en el segmento
    float dtheta = (theta_array[i]-theta_array[i-1])*percent_time; // Incremento de theta en el segmento
    float x_final = x_array[i-1]+dx_segment; // X del inicio del segmento mas el incremento en ese segmento
    float y_final = y_array[i-1]+dy_segment; // Y del inicio del segmento mas el incremento en ese segmento
    float theta_final = theta_array[i-1]+dtheta; // Theta del inicio del segmento mas el incremento en ese segmento

    if(fmod(control[1],delta_giro) != 0){
      i=0;
      int index_inicio_tramo2 = index_final_tramo1;
      do{
        i++;
      }while(t<(t_array[index_inicio_tramo2+i]-t_array[index_inicio_tramo2]));
      float t_tf2 = t + t_array[index_inicio_tramo2]; // Tiempo real total sumando el punto de inicio de esa trayectoria
      float percent_time2 = (t_tf2-t_array[i-1])/(t_array[i]-t_array[i-1]); // Porcentaje del total del segmento de tiempo en el que se encuentra
      float dx_segment2 = (x_array[i]-x_array[i-1])*percent_time2; // Incremento de x en el segmento
      float dy_segment2 = (y_array[i]-y_array[i-1])*percent_time2; // Incremento de y en el segmento
      float dtheta2 = (theta_array[i]-theta_array[i-1])*percent_time2; // Incremento de theta en el segmento
      float x_final2 = x_array[i-1]+dx_segment2; // X del inicio del segmento mas el incremento en ese segmento
      float y_final2 = y_array[i-1]+dy_segment2; // Y del inicio del segmento mas el incremento en ese segmento
      float theta_final2 = theta_array[i-1]+dtheta2; // Theta del inicio del segmento mas el incremento en ese segmento

      //Distancia relativa a ambas curvas, en porcentaje con respecto a la primera
      double percent_angle = fmod(control[1],delta_giro);
      x_out = x_final + (x_final2-x_final)*percent_angle;
      y_out = y_final + (y_final2-y_final)*percent_angle;
      theta_out = theta_final + (theta_final2-theta_final)*percent_angle;
    }else{
      x_out = x_final;
      y_out = y_final;
      theta_out = theta_final;
    }
    

    x_out += pose[0];
    y_out += pose[1];
    theta_out += pose[2];
    float pose_[3] = {x_out, y_out, theta_out};
    pose = pose_;

    return;
}


void tf_position (float x_, float y_, float theta_, float a, float b, float theta_0, float* pose_array){
  // a y b son las coordenadas iniciales leidas en la silla, se van a utilizar como el origen del nuevo sistema de referencia
  // x_ posicion actual
  // y_ posicion actual
  // theta_ posicion actual
  // a, b, theta_0 son las coordenadas y el angulo del sistema de referencias al que queremos convertir
  // pose_array vector de 3 floats que almacena la pose transformada
  float theta_0_rad;
  theta_0_rad = (PI/180)*theta_0;
  float x_tf;
  float y_tf;
  float theta_tf;
  theta_tf = theta_ - theta_0;
  x_tf = x_*cos(-theta_0_rad) - y_*sin(-theta_0_rad) - a*cos(-theta_0_rad) + b*sin(-theta_0_rad);
  y_tf = x_*sin(-theta_0_rad) + y_*cos(-theta_0_rad) - a*sin(-theta_0_rad) - b*cos(-theta_0_rad);
  if(fabs(x_tf)<0.000001){
    x_tf = 0;
  }
  if(fabs(y_tf)<0.000001){
    y_tf = 0;
  }
  //x_tf = (x_ - a)*cos(theta_0_rad) + (y_ - b)*sin(theta_0_rad);
  //y_tf = -(x_ - a)*sin(theta_0_rad) + (y_ - b)*cos(theta_0_rad);
  //x_tf = x_0 + x_*cos(theta_ - theta_0) - y_*sin(theta_ - theta_0);
  //y_tf = y_0 + x_*sin(theta_ - theta_0) + y_*cos(theta_ - theta_0);
  // std::cout << "x: " << x_ << " ,y: " << y_ << " ,theta: "<< theta_ << " // Coordenadas en el sistema de referencia original" << std::endl;
  // std::cout << "a: " << a << " ,b: " << b << " ,theta_0: "<< theta_0 << " // Sistema de referencia con respecto a la posicion inicial del robot" << std::endl;
  // std::cout << "xtf: " << x_tf << " ,ytf: " << y_tf << " ,thetatf: "<< theta_tf << " // Coordenadas transformadas al nuevo sistema de referencia" << std::endl;
  pose_array[0] = x_tf;
  pose_array[1] = y_tf;
  pose_array[2] = theta_tf;
  return;
}
