//Autor: RAUL CARRERO RAMOS 53869 56EE

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_camera.h>
#include "Arduino.h"
#include "driver/rtc_io.h"
#include <math.h>
#include <HTTPClient.h>
#include <ESP32Time.h>

//DEFINICION DE MACROS

#define CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM 32          //PIN PARA ENCENDER Y APAGAR LA CAMARA
#define RESET_GPIO_NUM -1         //REINICIO DE LA CAMARA
#define XCLK_GPIO_NUM 0           //RELOJ NECESARIO PARA LA OPERACION DE LA CAMARA
#define SIOD_GPIO_NUM 26          //COMUNICACION SERIE ENTRE MICRO Y LA CAMARA
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22           //RELOJ PARA SINCRONIZAR LA TRANSMISION DE PIXELES
#define TIEMPO_TEMPORIZADOR 1.5    //SE TOMA UNA FOTO CADA 1 MIN-2 MIN

ESP32Time RTC(3600);               //REAL TIME CLOCK (GMT+1 HORA ESPANHA)

//SSID Y PASSWORD DE LA RED DE CONEXION
const char *ssid = "TU-SSID";              
const char *password = "TU-PASSWORD";    

//SERVIDOR DE VISUALIZACION DE IMAGEN
const char *nombre_servidor = "servidor";                    
const int puerto_servidor = 80;

//URL DE LA API QUE RECOGE LA BASE DE DATOS Y KEY DE ACCESO
const char *url = "http://vps247.cesvima.upm.es/addData";     
const char *key = "KEY-DEL-SERVIDOR";

AsyncWebServer servidor(puerto_servidor);

//VARIABLES GLOBALES PARA IMAGEN EN JPEG Y BMP
size_t TamImagen;
uint8_t *pixeles;

size_t TamImagenBMP;
uint8_t *pixelesBMP;

//ESTRUCTURA DE DATOS DE ENVIO A LA API
struct datos {

  int id_sensor;    
  String timestamp;
  String latitud;
  String longitud;
  int orientacion;
  int inclinacion;
  String tipo_medida;
  String valor;
} datos = {6003, "2024-05-23T12:37:30", "40.717823", "-3.127636", 0, 0, "SVF", "0"};

//VARIABLES GLOBALES PARA TEMPORIZADOR 
unsigned long int t1 = 0;
unsigned long int intervalo = TIEMPO_TEMPORIZADOR*60*1000;       

void configuracion_camara() {

  //INICIALIZACION DE LA CAMARA
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;        //FRECUENCIA DE RELOJ 20 MHz
  config.pixel_format = PIXFORMAT_JPEG;  //FORMATO DE IMAGEN JPEG

  config.frame_size = FRAMESIZE_SVGA;    //800 x 600
  config.jpeg_quality = 12;           
  config.fb_count = 1;

  //CONTROL DE ERRORES

  esp_err_t init = esp_camera_init(&config);

  if (init != ESP_OK) {

    Serial.printf("Error en inicializacion de la camara %d\n", init);
  } else {

    Serial.printf("Camara iniciada correctamente\n");
  }
}

void conexion_wifi() {

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {

    Serial.print(".");
  }
  Serial.printf("\nConectado a WiFi %s con direccion IP: ", ssid);
  Serial.println(WiFi.localIP());
}

void captura_imagen() { 

  int terminado = 0;

  camera_fb_t *fb = NULL;
  uint8_t encabezado_bmp[54];  //54 BYTES DE DESPLAZAMIENTO DEL ENCABEZADO DEL ARCHIVO BMP

  fb = esp_camera_fb_get();

  if (!fb) {

    Serial.println("\nError al capturar la imagen");
  }else {

    Serial.println("\nImagen capturada correctamente");
  }

  //ACCESO A LOS PIXELES DE LA IMAGEN EN JPEG
  TamImagen = fb->len;
  pixeles = fb->buf; 

  //VARIABLES NECESARIAS PARA ALMACENAR LAS COMPONENTES RGB DE LA IMAGEN
  uint8_t R = 0;
  uint8_t G = 0;
  uint8_t B = 0;

  //PROCESADO DE LA IMAGEN

  int centro_x = fb->width/2;
  int centro_y = fb->height/2;

  int num_coronas = 6;
  int Ncoronas = num_coronas;                            //VARIABLE PARA PASARLA A LA FUNCION DE SVF
  float paso = fb->height / (float)(2.0*num_coronas);    //PARTE DE LA IMAGEN SE PIERDE PUESTO QUE NO ES UNA IMAGEN CIRCULAR
  float radios[num_coronas];
  int pixeles_Icorona[num_coronas];                      //VECTOR PARA ALMACENAR EL NUMERO DE PIXELES DE CADA CORONA
  int Ipixeles = 0;                                      //VARIABLE PARA ALMACENAR EN EL VECTOR
  int pixeles_cielo_Icorona[num_coronas];                //VECTOR PARA ALMACENAR EL NUMERO DE PIXELES DE CIELO DE CADA CORONA
  int Ipixeles_cielo = 0;                                //VARIABLE PARA ALMACENAR EN EL VECTOR DE PIXELES DE CIELO
  int posicion_pixel = 0;
  
  for(int i = 1; i <= num_coronas; i++) {                //INICIALIZACION DEL VECTOR DE RADIOS 

    radios[i-1] = paso*i;
    pixeles_Icorona[i-1] = 0;
    pixeles_cielo_Icorona[i-1] = 0;
  }

  float radio_interno = 0.0;
  float radio_externo = radios[0];
  float radio_aux;

  int i = 1;

  //ACCESO A LOS PIXELES DE LA IMAGEN EN BMP (PROCESAMIENTO DE LA IMAGEN)

  frame2bmp(fb, &pixelesBMP, &TamImagenBMP);  //SOLO ACEPTA CONVERSION DE JPEG A BITMAP SI FRAMESIZE_SVGA (800 x 600)

  Serial.printf("Tamanho imagen en bytes en formato JPEG: ");
  Serial.println(TamImagen);
  Serial.printf("Tamanho imagen en bytes en formato BMP: ");
  Serial.println(TamImagenBMP);

  while(num_coronas != 0) {

    //HAY QUE RECORRER LA IMAGEN TANTAS VECES COMO CORONAS HAYA

    for(int y = 0; y < fb->height; y++) {                                             //SE RECORRE CADA PIXEL DE LA IMAGEN Y SE CALCULA LA DISTANCIA AL CENTRO

      for(int x = 0; x < fb->width; x++) {

        float distancia_centro = sqrt(pow(x - centro_x, 2) + pow(y - centro_y, 2));
            
        if(distancia_centro >= radio_interno && distancia_centro < radio_externo) {   //PIXEL PERTENECE A LA CORONA i

          posicion_pixel = sizeof(encabezado_bmp) + (y * fb->width + x)*3;            //CALCULO DE LA POSICION LINEAL DEL PIXEL EQUIVALENTE A NOMENCLATURA (Y, X)

          B = pixelesBMP[posicion_pixel];           //COMPONENTE AZUL

          G = pixelesBMP[posicion_pixel + 1];       //COMPONENTE VERDE

          R = pixelesBMP[posicion_pixel + 2];       //COMPONENTE ROJA 

          //SE COMPRUEBA SI LOS PIXELES SON DE CIELO EN BASE A COMPONENTES RGB

         if(((R >= 60 && R <= 200) && (G >= 80 && G <= 230) && (B >= 140)) ||                                                          //AZULES                                                                                                              
            ((R >= 50 && R <= 100) && (G >= 70 && G <= 130) && (B >= 80 && B <= 139)) ||                                               //AZULES VERDOSOS OSCUROS            
            ((R >= 190) && (G >= 200) && (B >= 165)) ||                                                                                //BLANCOS
            ((R >= 70 && R <= 220) && (G >= 85) && (B >= 70 && B <= 220) && ((R - G) <= 25) && ((R - B) <= 25) && ((B - G) <= 25))) {  //GRISES

            pixelesBMP[posicion_pixel] = 0;         //SE MARCAN EN NEGRO PARA COMPROBACION EN LA IMAGEN
            pixelesBMP[posicion_pixel + 1] = 0; 
            pixelesBMP[posicion_pixel + 2] = 0; 
            Ipixeles_cielo++;                       //SE AUMENTA EL CONTADOR DE PIXELES DE CIELO
          }
          
          Ipixeles++;                               //NUMERO DE PIXELES TOTALES DE LA CORONA i
        }        
      }
    }
    
    Serial.printf("Corona: ");
    Serial.println(num_coronas);
    Serial.printf("Numero de pixeles: ");
    pixeles_Icorona[i-1] = Ipixeles;
    Serial.println(pixeles_Icorona[i-1]);
    Serial.printf("Numero de pixeles de cielo: ");
    pixeles_cielo_Icorona[i-1] = Ipixeles_cielo;
    Serial.println(pixeles_cielo_Icorona[i-1]);
    Serial.println(radio_interno);
    Serial.println(radio_externo);
    num_coronas--;
    radio_aux = radio_externo;
    radio_interno = radio_aux;
    radio_externo = radios[i];
    i++;
    Ipixeles = 0;
    Ipixeles_cielo = 0;
  }

  esp_camera_fb_return(fb);  //LIBERA EL BUFFER DE LA IMAGEN

  calculo_SVF(pixeles_cielo_Icorona, pixeles_Icorona, Ncoronas);

  Serial.println("Procesamiento de imagen completado");
}

void envio_imagen(AsyncWebServerRequest *solicitud) { 
  
  solicitud->send_P(200, "image/bmp", pixelesBMP, TamImagenBMP); 

  Serial.println("Imagen enviada correctamente");
}

void iniciar_servidor() {

  //ENVIO DE IMAGEN A SERVIDOR WEB DE VISUALIZACION
  servidor.on("/", HTTP_GET, envio_imagen);
  servidor.begin();  //INICIA EL SERVIDOR
}

void calculo_SVF(int vector1[], int vector2[], int ncoronas) {

  double SVF = 0.0;

  for(int i = 1; i <= ncoronas; i++) {

    SVF = SVF + sin((M_PI*(2.0*(double)i - 1.0)) / (double)(2.0*ncoronas))*(((double)vector1[i-1]*2.0*M_PI) / (double)vector2[i-1]);
  }

  SVF = (1.0 / (4.0*(double)ncoronas))*SVF;
  Serial.print("El valor de Sky View Factor es: ");
  Serial.println(SVF);
  datos.valor = (String)SVF;
}

void envio_SVF_api() {

  HTTPClient http;
  http.begin(url);
  datos.timestamp = RTC.getTime("%FT%T");
  
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("token", key);

  String payload = "id_sensor=" + String(datos.id_sensor) +
                    "&timestamp=" + datos.timestamp +
                    "&latitud=" + datos.latitud +
                    "&longitud=" + datos.longitud +
                    "&orientacion=" + String(datos.orientacion) +
                    "&inclinacion=" + String(datos.inclinacion) +
                    "&tipo_medida=" + datos.tipo_medida +
                    "&valor_medida=" + datos.valor;

    int codigo_respuesta_http = http.POST(payload);        //ENVIO DE LA SOLICITUD POST

    if (codigo_respuesta_http > 0) {                       //VERIFICA LA RESPUESTA

      String respuesta = http.getString();
      Serial.print("Codigo de respuesta: ");
      Serial.println(codigo_respuesta_http);
      Serial.println(respuesta);
    } else {

      Serial.print("Error en la solicitud POST: ");
      Serial.println(codigo_respuesta_http);
    }

    http.end();
}

void setup() {

  Serial.begin(460800);                     //PARA COMUNICARSE CON SERIAL MONITOR

  delay(2000);                              //PARA DARLE TIEMPO A SERIAL MONITOR A INICIARSE

  RTC.setTime(30, 59, 11, 24, 5, 2024);     //SE ESTABLECE LA HORA DE MERIDIANO GREENWICH 

  configuracion_camara();
        
  conexion_wifi();                          //CONEXION A RED WIFI

  iniciar_servidor();
}

void loop() {

  unsigned long int tiempo_actual = millis();

  if((tiempo_actual - t1) >= intervalo) {

    t1 = tiempo_actual;

    captura_imagen();

    envio_SVF_api();

    free(pixelesBMP);
  }
}







