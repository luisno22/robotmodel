#pragma once

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/byte_multi_array.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/float32_multi_array.hpp>
#include <example_interfaces/msg/int32.hpp>


using std::placeholders::_1;

//#include <iostream>
#include <ctime>
#include <vector>
#include <cmath>
#include <math.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include "Yml.hpp"



#define delta_giro		10
#define giro_max		  20  // 45 grados a cada lado
std::string name = "300high_max20_delta10";
#define d_max			    3
#define tmuestreo		  0.5
#define v             25  // velocidad de movimiento de porcentaje del maximo
#define PI            3.14159265359  // Constante pi
#define iteraciones   1
#define delay         10000
std::string yml_path = "C:/configuraciones/robotmodel/";


struct output {
	float pose[3];
	double time;
	int step;
  float d;
};

// using OnShutdownCallback = std::function<void()>;

void tf_position (float, float, float, float, float, float, float *);
void write_yml_groups(std::ofstream& , int, float, float);

/**
 * @brief Node Class used for robot model
 * @param  none
 * @return  none
 */
class RobotModelNode : public rclcpp::Node{
  public:
      std::vector<output> datos; // struct que almacena la salida: pose, time, distancia y numero de pasos actual
      // numero de paso actual
      int step_now = 0;

      // Variables de tiempo
      std::chrono::time_point<std::chrono::steady_clock> t0;
      std::chrono::time_point<std::chrono::steady_clock> t;

      //Posicion inicial
      float xp[3];

      //simulaci�n
      int option = 0; //valor entre 0 y 3 que indica que parte del movimiento se est� realizando: 0{adelante x+}, 1{atras x+}, 2{atras x-}, 3{adelante x+}
      int ciclo = 0;  //valor que indica la cantidad de veces que se han repetido las 2 partes del movimiento
      int arco = 0;  // Valor que indica el numero de arco que se esta realizando para guardarlo despues en el yml
      int option_last = 0;
      int ciclo_last = 0;
      int iteracion = 0;

      //Control inicial
      float u[2];

      // Posicion inicial con respecto al sistema de referencia por defecto
      float a;
      float b;
      float theta_0;
      bool first_iteration = true;
      
      // Distancia acumulada, inicializada en 0
      double d = 0;

      // Variable que indica el final del modelo
      bool finished = false;

      // Variable auxiliar para almacenar un dato tipo output
      output aux;

      // Variable para pruebas
      int test = 0;

      // Variable de archivo yml para guardar el modelo
      std::ofstream yml_parameters;

      // Variable que almacena los indices donde se produce un cambio de opcion
      std::vector<int> index;

      RobotModelNode(): Node("RobotModelNode"){
          RCLCPP_INFO(this->get_logger(), "Initializing RobotModelNode ... WheelChair tested");
          std::cout<<"constructor";
          RobotModelControlPub = this->create_publisher<example_interfaces::msg::Float64MultiArray>("/control_from_robotModel", 0);
          posVehSub = this->create_subscription<example_interfaces::msg::Float64MultiArray>("/posVeh", 0, bind(&RobotModelNode::posVeh_subscriber_callback, this, _1));
          // rclcpp::on_shutdown(on_shutdown_callback);
          aux.pose[0] = 0;
          aux.pose[1] = 0;
          aux.pose[2] = 0;
          aux.step = 0;
          aux.time = 0;
          datos.push_back(aux);
          index.push_back(0);
          t0 = std::chrono::steady_clock::now();
          u[0] = v; //velocidad
          u[1] = giro_max - delta_giro*ciclo; //giro_max - delta_giro * ciclo; //giro inicial
          step_now++;
          yml_parameters.open(yml_path + "model_" + name + ".yml",std::ios::trunc);
          yml_parameters << "name: " + name + "\nd_max: " + std::to_string(d_max) + "\ngiro_max: " + std::to_string(giro_max) + "\ndelta_giro: " + std::to_string(delta_giro) +"\niteraciones: " + std::to_string(iteraciones) + "\ndelay: " + std::to_string(delay) + "\nvelocidad: " + std::to_string(v) << std::endl;
          std::ofstream myfile("file.txt");
          myfile << "test" << std::endl;
          myfile.close();
      }

  private:
    /**
     * @brief Subscriber callback. Will subscribe to a topic called /posVeh and will do a simple model without correcting errors
     * @param  msg Mensaje publicado en el topic /posVeh que contiene un array de 3 floats, [0] x(m), [1] y(m), [2] theta(rad)
     * @return  none
     */
    void posVeh_subscriber_callback(const example_interfaces::msg::Float64MultiArray::SharedPtr msg){
      // En caso de haber un cambio de sentido hace una parada de 10 segundos para reducir la cantidad del error producido por el tambaleo del LIDAR
      if((ciclo_last != ciclo) || (option_last != option)){
        //std::cout<<"cambio de ciclo u opcion"<<std::endl;
        // Publica valores de control 0 para detener el movimiento
        example_interfaces::msg::Float64MultiArray to_publish;
        to_publish.data = {0,0};
        RobotModelControlPub->publish(to_publish);
        // Calcula el tiempo transcurrido desde la detención, es decir, desde la lectura del ultimo valor de tiempo
        t = std::chrono::steady_clock::now();
        double time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t - t0).count() - datos.back().time;

        // Una vez superado el tiempo de espera se devuelve la ejecucion al punto de first_iteration y se publican los valores de control correspondientes al momento actual
        if(time_elapsed>delay){
          std::cout<<"delay"<<std::endl;
          first_iteration=true;
          option_last = option;
          ciclo_last = ciclo;
          if(option==1){
            u[0]=-v*3;
            u[1]=-(giro_max - delta_giro*ciclo)*2;
            RobotModelControlPub->publish(to_publish);
          }else if(option==0){
            u[0] = v;
            u[1] = giro_max - delta_giro*ciclo;
            RobotModelControlPub->publish(to_publish);
          }
        }else{
            RobotModelControlPub->publish(to_publish);
          return;
        }
      }

      // Ejecucion en cada primera iteracion de cada cambio de sentido
      if(first_iteration){
        // Inicializacion de variables de posicion y distancia
        d = 0;
        example_interfaces::msg::Float64MultiArray::SharedPtr actual_position = msg;
        // Posicion inicial de cada tramo (a,b,theta_0)
        a = actual_position->data[0]; //pos X (m)
        b = actual_position->data[1]; //pos Y (m)
        theta_0 = actual_position->data[2] * 180/PI; //Theta (grados)
        first_iteration = false;
        std::cout<<"first iteration\n";
        option_last = option;
        ciclo_last = ciclo;
      }else if(!finished){ // Ejecucion normal del codigo
        // Posicion actual
        example_interfaces::msg::Float64MultiArray::SharedPtr actual_position = msg;
        xp[0] = actual_position->data[0]; //pos X
        xp[1] = actual_position->data[1]; //pos Y
        xp[2] = actual_position->data[2] *180/PI; //Theta en grados 

        // Instante de tiempo actual
        t = std::chrono::steady_clock::now();
        // Intervalo de tiempo transcurrido desde el inicio de la ejecucion
        double t_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(t - t0).count();
        // Posicion transformada a (0,0,0) con origen en la posicion inicial del movimiento
        float tf_pose[3];
        tf_position(xp[0], xp[1], xp[2], a, b, theta_0, tf_pose);
        aux.pose[0] = tf_pose[0];
        aux.pose[1] = tf_pose[1];
        aux.pose[2] = tf_pose[2];
        std::cout<<aux.pose[1]<<std::endl;

        // Tiempo y numero de paso actual
        aux.step = step_now;
        aux.time = t_milliseconds;

        // Añade los datos del paso actual al vector de datos
        datos.push_back(aux);
        std::cout<<"step_now: "<<step_now<<std::endl;
        std::cout<<"theta: " <<xp[2]<<std::endl;

        // Incrementos de posicion en cada direccion e incremento de la distancia euclidiana
        float dx = datos[step_now].pose[0] - datos[step_now-1].pose[0]; //Incremento de x con respecto a la posicion anterior
        float dy = datos[step_now].pose[1] - datos[step_now-1].pose[1]; //Incremento de y con respecto a la posicion anterior
        float dd = hypot(dx, dy);                                       // Incremento de la distancia euclidiana con respecto a la posicion anterior
        std::cout<<"dd: "<<dd<<std::endl;

        // Evita variaciones minimas de posicion debido al error de medicion
        if(dd>0.001){
          d += dd;
        }
        std::cout<<"d: "<<d<<std::endl;

        // Distancia recorrida en su trayectoria desde el origen
        if(d<0.000001){
          d=0;
        }
        datos.back().d = d;
        std::cout<<"distance: "<<datos[step_now].d<<std::endl;

        option_last = option;
        ciclo_last = ciclo;
        // Condiciones de distancia maxima superada
        if(d > d_max){
          if (u[0] >= 0) { // Superada con velocidad positiva, es decir, con sentido de movimiento positivo
            dd = 0;
            option = 1;
            write_yml_groups(yml_parameters, arco, u[0], u[1]);
            u[0] = -v*3;
            u[1] = -(giro_max - delta_giro*ciclo)*2; // Los motores marcha atras son mas lentos lo que provocan que el angulo de giro hacia adelante y hacia atras no sea el mismo
            std::cout<<"\n\n\n\n---------cambio de sentido a la opcion = 1, marcha atras--------\n\n\n\n\n"<<std::endl;
            arco++;
            step_now++;
            aux.pose[0] = 0;
            aux.pose[1] = 0;
            aux.pose[2] = 0;
            aux.step = step_now;
            aux.time = t_milliseconds;
            datos.push_back(aux);
            index.push_back(step_now);
          }else if(u[0] < 0){ // Superada con velocidad negativa
            dd = 0;
            option = 0;
            write_yml_groups(yml_parameters, arco, u[0], u[1]);
            ciclo++;
            u[0] = v;
            u[1] = giro_max - delta_giro*ciclo;
            std::cout<<"\n\n\n\n------------------ cambio de sentido a la opcion = 0, marcha adelante --------------------\n\n\n\n"<<std::endl<<std::endl;
            step_now++;
            arco++;
            aux.pose[0] = 0;
            aux.pose[1] = 0;
            aux.pose[2] = 0;
            aux.step = step_now;
            aux.time = t_milliseconds;
            datos.push_back(aux);
            index.push_back(step_now);
            
            
            int max_ciclos = (giro_max+giro_max)/delta_giro;
            if (ciclo > max_ciclos) {
                std::cout << giro_max << " " << u[1];
                iteracion++;
                ciclo = 0;
                option = 0;
                u[0] = v;
                u[1] = giro_max - delta_giro*ciclo;
                first_iteration = true;
                if(iteracion >= iteraciones){
                  std::cout << "finished";
                  finished = true;
                }
            }
          }
        }
        std::cout << "d y dd despues del cambio de opcion: "<<d<<" "<<dd<<"\n";

        std::cout<< "control velocidad: " << u[0] << " , control giro: " << u[1] << std::endl << std::endl;
        example_interfaces::msg::Float64MultiArray to_publish;
        to_publish.data = {u[0],u[1]};
        step_now++;
        RobotModelControlPub->publish(to_publish);
      }else{
        yml_parameters.close();
        std::cout<<"exit"<<std::endl;
        example_interfaces::msg::Float64MultiArray to_publish;
        to_publish.data = {0,0};
        RobotModelControlPub->publish(to_publish);
        step_now++;

        //Define las variables de archivo para guardar las posiciones en 2 archivos de texto
        std::ofstream myfilex;
        std::ofstream myfiley;
        std::ofstream myfiletheta;
        std::ofstream myfile_t;
        std::ofstream myfile_d;
        myfilex.open("vector_x_" + name + ".txt",std::ios::trunc);
        myfiley.open("vector_y_" + name + ".txt",std::ios::trunc);
        myfiletheta.open("vector_theta_" + name + ".txt",std::ios::trunc);
        myfile_t.open("vector_t_" + name + ".txt",std::ios::trunc);
        myfile_d.open("vector_d_" + name + ".txt",std::ios::trunc);
        if(myfilex.is_open() && myfiley.is_open()){
          std::cout<<"Todos los archivos abiertos correctamente, procediendo a reescribirlos"<<std::endl;
        }
        

        // Almacena todas las variables en sus correspondientes archivos de texto
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> theta;
        std::vector<double> time;
        std::vector<double> dist;
        std::vector<int> stepv;
        for (int j = 0; j < datos.size(); j++) {
          if ((datos[j].pose[0] < 4000000) & (datos[j].pose[0] > -4000000) & (datos[j].pose[1] < 4000000) & (datos[j].pose[0] > -4000000)) {
            //matrix[0][j] = datos[j].pose[0];
            myfilex << datos[j].pose[0] << std::endl;
            //matrix[1][j] = datos[j].pose[1];
            myfiley << datos[j].pose[1] << std::endl;
            myfiletheta << datos[j].pose[2] << std::endl;
            myfile_t << datos[j].time << std::endl;
            myfile_d << datos[j].d << std::endl;
            std::cout<<datos[j].pose[1]<<std::endl;
            x.push_back(datos[j].pose[0]);
            y.push_back(datos[j].pose[1]);
            theta.push_back(datos[j].pose[2]);
            time.push_back(datos[j].time);
            dist.push_back(datos[j].d);
            stepv.push_back(datos[j].step);
          }
        }
        myfilex.close();
        myfiley.close();
        myfiletheta.close();
        myfile_t.close();
        myfile_d.close();
        std::cout<<"Todos los archivos guardados y cerrados, FIN DEL PROGRAMA"<<std::endl;

        // Abre un archivo yml y almacena todos los datos obtenidos durante la ejecucion
        //Get yml file
        Yml yml_model("configuraciones/robotmodel/model_" + name + ".yml");
        //Load yml file
        yml_model.LoadConfiguration();

        // Recorre el vector que indica los indices en los que hay cambios de sentido o de ciclo
        for(int i=0;i<index.size();i++){
          std::vector<double> x_sub;
          std::vector<double> y_sub;
          std::vector<double> theta_sub;
          std::vector<double> t_sub;
          std::vector<double> d_sub;
          std::vector<int> stepv_sub;
          // Crea subvectores para almacenar los datos de unicamente un tramo
          for(int j=index[i];j<index[i+1];j++){
            x_sub.push_back(x[j]);
            y_sub.push_back(y[j]);
            theta_sub.push_back(theta[j]);
            t_sub.push_back(time[j] - time[index[i]]);
            d_sub.push_back(dist[j]);
            stepv_sub.push_back(stepv[j]);
          }
          // Añade los datos de ese tramo al yml
          yml_model.SetDoubleArray("arco_"+std::to_string(i)+"-vectorx",x_sub);
          yml_model.SetDoubleArray("arco_"+std::to_string(i)+"-vectory",y_sub);
          yml_model.SetDoubleArray("arco_"+std::to_string(i)+"-vectortheta",theta_sub);
          yml_model.SetDoubleArray("arco_"+std::to_string(i)+"-vectortime",t_sub);
          yml_model.SetDoubleArray("arco_"+std::to_string(i)+"-vectordistance",d_sub);
          yml_model.SetIntArray("arco_"+std::to_string(i)+"-step",stepv_sub);
        }
        yml_model.Save();
        exit(0);
      }
    }



    // std::function<void ()> on_shutdown_callback = []() {
    //   example_interfaces::msg::Float64MultiArray control;
    //   control.data = {0,0};
    //   std::cout<<"shutdown callback -> published control: {" << control.data[0] << ", " << control.data[1] << "}" <<std::endl;
      
    // };
  

    rclcpp::Publisher<example_interfaces::msg::Float64MultiArray>::SharedPtr RobotModelControlPub; // Publisher can data
    rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr posVehSub; //Subscriber to control can

};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotModelNode>());
  rclcpp::shutdown();
  return 0;
}

/// @brief Transforma coordenadas desde el sistema de coordenadas global con origen en (0,0) al nuevo sistema de referencia con coordenadas de origen (a,b) y orientacion theta_0
/// @param x_ Coordenada x en el sistema de referencia global
/// @param y_ Coordenada y en el sistema de referencia global
/// @param theta_ Orientación theta en el sistema de referencia global en grados
/// @param a Coordenada x origen del nuevo sistema de referencia
/// @param b Coordenada y origen del nuevo sistema de referencia
/// @param theta_0 Orientacion theta origen del nuevo sistema de referencia en grados
/// @param pose_array Array de 3 floats que almacenara la pose transformada del vehiculo
void tf_position (float x_, float y_, float theta_, float a, float b, float theta_0, float* pose_array){
  float theta_0_rad;
  theta_0_rad = (PI/180)*theta_0; // theta_0 en radianes
  float x_tf;
  float y_tf;
  float theta_tf;
  theta_tf = theta_ - theta_0;
  x_tf = x_*cos(-theta_0_rad + PI) - y_*sin(-theta_0_rad + PI) - a*cos(-theta_0_rad + PI) + b*sin(-theta_0_rad + PI);
  y_tf = x_*sin(-theta_0_rad + PI) + y_*cos(-theta_0_rad + PI) - a*sin(-theta_0_rad + PI) - b*cos(-theta_0_rad + PI);
  if(fabs(x_tf)<0.000001){
    x_tf = 0;
  }
  if(fabs(y_tf)<0.000001){
    y_tf = 0;
  }
  pose_array[0] = x_tf;
  pose_array[1] = y_tf;
  pose_array[2] = theta_tf;
  return;
}


/// @brief Escribe en un archivo yml los grupos y subgrupos necesarios a almacenar
/// @param file Archivo a escribir
/// @param arco Numero de arco
/// @param velocidad Velocidad en un arco concreto
/// @param giro Giro en un arco concreto
void write_yml_groups(std::ofstream& file, int arco, float velocidad, float giro){
  file << "arco_"+std::to_string(arco)+": \n giro: "+std::to_string(giro)+" \n velocidad: "+std::to_string(velocidad)+"\n vectorx:\n  - \n vectory:\n  - \n vectortheta:\n  - \n vectortime:\n  - \n vectordistance:\n  - \n step:\n  - \n";
}
