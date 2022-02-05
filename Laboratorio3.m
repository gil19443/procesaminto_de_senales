%% Script laboratorio 3 
% Nombre: Carlos Gil 
% Carnet: 19443
% Curso: IE3032-Procesamiento de señales 
% Sección: 20
% Tarea: Laboratorio # 3 

%% Ejercicio # 1 
% Corra la sección 5 veces y calcule el promedio de los tiempos desplegados. Note que la
% instrucción para definir la matriz H está comentada. Por lo tanto, H irá creciendo en
% cada iteración. 

% Después, descomente la línea H = zeros(N, M); y corra la sección 5 veces más. Calcule el
% promedio de los tiempos desplegados. Si la diferencia de los tiempos promedio no es
% significativa con y sin la pre-definición de H, pruebe aumentar las dimensiones de la
% matriz (N y M).

% En el script que subirá a Canvas: Escriba los tiempos promedio obtenidos.

clear H
N = 10000;
M = 15000;
H = zeros(N, M);
if i<1 %condicional para que el indice del arreglo solo se inicialice una vez 
    i=1; 
    AV = zeros(1,10);%solo se crea una vez la matriz 
end
tic
for m = 1:M
    for n = 1:N
        H(n,m) = 1/(n+m-1);
    end
end
AV(i) = toc %se guarda el tiempo en un arreglo 
%los primeros 5 valores son con la instruccion comentada 
temp_prom = (AV(1)+AV(2)+AV(3)+AV(4)+AV(5))/5;
%los otros 5 valores son con la instruccion sin comentar 
temp_prom2 = (AV(6)+AV(7)+AV(8)+AV(9)+AV(10))/5;
i = i +1;
fprintf('El promedio con la linea comentada es: %f \n',temp_prom);
fprintf('El promedio sin la linea comentada es: %f \n',temp_prom2);
fprintf('La diferencia entre los timepos promedio es: %d \n',temp_prom-temp_prom2);
%el tiempo promedio con la linea comentada fue de: 1.412594 s
%el tiempo promedio con la linea sin comentar fue de: 0.342155 s
%la diferencia fue de: 1.070438e+00 la cual es significativa s

%% Ejercicio 2
v1 = [1 2 3 4 5];
v2 = [6 7 8 9 10];
v3 = [1 0 11 0 1];
%Pruebas realizadas al llamar a las funciones 
[mu1, s1, M1, m1, Suma1] = PS_Lab3_fun1(v1);
[mu2, s2, M2, m2, Suma2] = PS_Lab3_fun1(v2)
PS_Lab3_fun1(v3);

%% Ejercicio 3
%matrices y vectores con al menos un cero, un numero positivo y un numero 
%negativo
v4 = [-1:5];
m1 = [zeros(1,7);-1:5];
%Puebas a la funcion con ciclo for y sin ciclo for 
[num_ceros, num_neg, num_pos] = PS_Lab3_fun2(v4, 0);
[num_ceros1, num_neg1, num_pos1] = PS_Lab3_fun2(m1, 1);
%% Ejercicio 4
%matriz de 4000x6000 de numeros aleatorios entre 0 y 1
r = rand(4000,6000);
%matriz de numeros complejos con parte real e imaginaria entre 0 y 1 
ri = complex(r,r);
%vectores para guardar los valores de tiempo 
v1_prom = [zeros(1,5)];
v2_prom = [zeros(1,5)];
for c = 1:5 %ciclo para ejecutar la funcion con ciclo for 5 veces y calcular
    tic;  %el tiempo promedio 
    PS_Lab3_fun2(ri, 0);
    v1_prom(c) = toc; %se guarda el valor del tiempo en el arreglo 
end
%se calcula el valor promedio del arreglo con los timpos 
fprintf('El tiempo promedio con ciclos for fue: %f \n', mean(v1_prom));
%mismo caso que la rutina anterior solo que la funcion sin ciclo for 
for d = 1:5
    tic;
    PS_Lab3_fun2(ri, 1);
    v2_prom(d) = toc;
end
fprintf('El tiempo promedio sin ciclos for fue: %f \n', mean(v2_prom));
%El tiempo promedio menor fue el resultado de llamar a la funcion sin que 
%esta utilice ciclos for 
%% Ejercicio 5
%se abre la imagen del osciloscopio en una figura 
imshow('Imagen_de_prueba1.png');
%se guarda en imdata los valores de los pixeles de la imagen del
%osciloscopio 
imdata = imread('Imagen_de_prueba1.png');

%% Ejercicio 6
%Instrucciones de prueba usadas en el laboratorio para agarrar los valores
%utiles del archivo csv y guardarlos en el archivo vectores.mat
% figure(2);
% plot(VarName5);
% y_util = VarName5(303:1399);
% x_util = VarName4(303:1399);
% save('vectores.mat', 'y_util', 'x_util');
%se cargan los vectores columna de los valores de la se;al 
load('vectores.mat');
figure(3);
plot(x_util, y_util); %se crea la grafica 
ylabel('voltaje (v)'); %se le colocan nombres a los ejes 
xlabel('tiempo (s)');
title('Señal del canal 1'); %se coloca el titulo 
%el numero de muestras es el numero de filas del vector columna 
fprintf('El número de muestras es: %d \n',size(x_util,1));
%el periodo de muestreo es el tiempo total medido dentro de la cantidad de
%mediciones tomadas 
periodo_m = (x_util(1097)-x_util(1))/size(x_util,1);
fprintf('El periodo de muestreo del osciloscopio es: %f \n',periodo_m);
fprintf('el voltaje máximo de la señal es: %f \n',max(y_util));
fprintf('el voltaje mínimo de la señal es: %f \n',min(y_util));
fprintf('El promedio de la señal es: %d \n',mean(y_util));
%Pregunta:
%¿Cómo se compara la señal observada en la imagen del osciloscopio con la graficada en
%Matlab? ¿Se parecen?
%La señal graficada en matlab, en comparacion con la señal tomada por el 
%osciloscopio, es extremadamente similar.
%¿Cuál es la amplitud de la señal? 
%la amplitud de la señal es de 2.8 V
%¿Cuál es la frecuencia de la señal?
%Para determinar la frecuencia de la señal, se utilizó la figura generada 
%por matlab. En la figura se pueden observar los valores de cada punto, 
%por lo que se seleccionaron los puntos de dos maximos consecutivos y se 
%restaron sus valores de tiempo,  lo que dio como resultado el periodo. 
%El valor obtenido fue de 0.176 s. Partiendo de este dato, se puede 
% utilizar la expresion de f = 1/T para determinar la frecuencai, 
% la cual dio como resultado 5.68 Hz.