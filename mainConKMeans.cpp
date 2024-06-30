#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <random>
#include <iomanip>
#include <limits>
#include <queue>
#include <unordered_map>
#include <cstdlib>
#include <ctime>

// Estructura para representar un microservicio
struct Microservicio {
    std::string descripcion;
    double ram;        // en MB
    double battery;    // en GCycles
    double inputSize;  // en MB
    bool critical;
};

// Estructura para representar un dron
struct Drone {
    int id;
    double posX;
    double posY;
    std::vector<int> microservicios; // Lista de microservicios que ofrece
};

// Estructura para asociar un usuario con su dron más cercano
struct UsuarioDronAsociado {
    int usuarioID;
    int dronID;
    double distancia;
};

// Estructura para representar un centroide
struct Centroide {
    double posX;
    double posY;
    double memoriaRestante;
    std::unordered_map<int, bool> microserviciosDesplegados;
    std::vector<int> microservicios; // Lista de microservicios que ofrece

    Centroide(double x = 0, double y = 0, double memoria = 4.0 * 1024)
        : posX(x), posY(y), memoriaRestante(memoria) {}

    bool operator==(const Centroide& other) const {
        return posX == other.posX && posY == other.posY;
    }
};

const double EARTH_RADIUS_KM = 6371.0;

// Función para convertir grados a radianes
double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

// Función para calcular la distancia entre dos puntos en 2D en metros
double calcularDistanciaEnMetros(double x1, double y1, double x2, double y2) { 
    double dLat = degToRad(y2 - y1); //Latitud es Y
    double dLon = degToRad(x2 - x1); //Longitud es X
    
    y1 = degToRad(y1);
    y2 = degToRad(y2);

    double a = sin(dLat/2) * sin(dLat/2) +
               cos(y1) * cos(y2) *
               sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return (EARTH_RADIUS_KM * c) * 1000;
}

// Asignar microservicios fijos a los drones
void asignarMicroserviciosFijos(std::vector<Drone>& drones) {
    drones[0].microservicios = {1, 2};
    drones[1].microservicios = {3, 4};
    drones[2].microservicios = {1, 3};
    drones[3].microservicios = {2, 4};
    drones[4].microservicios = {1, 4};
    drones[5].microservicios = {2, 3};
    drones[6].microservicios = {1, 2};
    drones[7].microservicios = {3, 4};
    drones[8].microservicios = {1, 3};

    //Replicacion del 25%
    /*drones[0].microservicios = {1};
    drones[1].microservicios = {2};
    drones[2].microservicios = {3};
    drones[3].microservicios = {4};
    drones[4].microservicios = {1};
    drones[5].microservicios = {2};
    drones[6].microservicios = {3};
    drones[7].microservicios = {4};
    drones[8].microservicios = {1, 2, 3, 4};*/

    //Replicacion del 50%
    /*drones[0].microservicios = {1, 2};
    drones[1].microservicios = {1, 3};
    drones[2].microservicios = {1, 4};
    drones[3].microservicios = {2, 3};
    drones[4].microservicios = {2, 4};
    drones[5].microservicios = {3, 4};
    drones[6].microservicios = {1, 3};
    drones[7].microservicios = {2, 4};
    drones[8].microservicios = {1, 4};*/

    //Replicacion del 100%
    /*drones[0].microservicios = {1, 2, 3, 4};
    drones[1].microservicios = {1, 2, 3, 4};
    drones[2].microservicios = {1, 2, 3, 4};
    drones[3].microservicios = {1, 2, 3, 4};
    drones[4].microservicios = {1, 2, 3, 4};
    drones[5].microservicios = {1, 2, 3, 4};
    drones[6].microservicios = {1, 2, 3, 4};
    drones[7].microservicios = {1, 2, 3, 4};
    drones[8].microservicios = {1, 2, 3, 4};*/
}

// Calcula la distancia entre todos los pares de drones de un sistema
std::vector<std::vector<double>> calcularMatrizAdyacencia(std::vector<Drone>& drones) {
    int n = drones.size(); // Num de drones
    std::vector<std::vector<double>> matrizAdyacencia(n, std::vector<double>(n, std::numeric_limits<double>::max()));

    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            double distancia = calcularDistanciaEnMetros(drones[i].posX, drones[i].posY, drones[j].posX, drones[j].posY);
            matrizAdyacencia[i][j] = distancia;
            matrizAdyacencia[j][i] = distancia; // Asumiendo que el grafo es no dirigido
        }
    }
    return matrizAdyacencia;
}

// Encontrar camino más corto desde un nodo origen a todos los demás en grafo
std::pair<std::vector<double>, std::vector<int>> dijkstra(const std::vector<std::vector<double>>& grafo, int origen) {
    int n = grafo.size();
    std::vector<double> distancias(n, std::numeric_limits<double>::max());
    std::vector<int> predecesores(n, -1);
    distancias[origen] = 0;
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> cola;
    cola.push({0, origen});

    while (!cola.empty()) {
        auto [dist, u] = cola.top();
        cola.pop();
        if (dist > distancias[u])
            continue;
        for (int v = 0; v < n; ++v) {
            if (grafo[u][v] != std::numeric_limits<double>::max() && distancias[u] + grafo[u][v] < distancias[v]) {
                distancias[v] = distancias[u] + grafo[u][v];
                predecesores[v] = u;
                cola.push({distancias[v], v});
            }
        }
    }
    return {distancias, predecesores};
}

// Función k-means para agrupar drones
std::pair<std::vector<Drone>, std::vector<int>> kMeansClustering(std::vector<Drone>& drones, int k, const std::vector<int>& centroidIndices) {
    int n = drones.size();
    std::vector<Drone> centroides(k);

    // Inicializar centroides con índices dados
    for (int i = 0; i < k; ++i) {
        centroides[i] = drones[centroidIndices[i]];
    }

    std::vector<int> asignaciones(n, -1);
    bool convergencia = false;

    while (!convergencia) {
        convergencia = true;

        // Asignar cada dron al centroide más cercano
        for (int i = 0; i < n; ++i) {
            double minDist = std::numeric_limits<double>::max();
            int minIndex = -1;
            for (int j = 0; j < k; ++j) {
                double dist = calcularDistanciaEnMetros(drones[i].posX, drones[i].posY, centroides[j].posX, centroides[j].posY);
                if (dist < minDist) {
                    minDist = dist;
                    minIndex = j;
                }
            }
            if (asignaciones[i] != minIndex) {
                convergencia = false;
                asignaciones[i] = minIndex;
            }
        }

        // Recalcular los centroides
        std::vector<double> sumX(k, 0.0), sumY(k, 0.0);
        std::vector<int> count(k, 0);
        for (int i = 0; i < n; ++i) {
            int clusterId = asignaciones[i];
            sumX[clusterId] += drones[i].posX;
            sumY[clusterId] += drones[i].posY;
            count[clusterId] += 1;
        }

        for (int j = 0; j < k; ++j) {
            if (count[j] != 0) {
                centroides[j].posX = sumX[j] / count[j];
                centroides[j].posY = sumY[j] / count[j];
            }
        }
    }

    return {centroides, asignaciones};
}

// Función para leer el archivo CSV con las solicitudes de los usuarios
std::vector<std::tuple<int, std::vector<int>, double, double>> leerArchivoCSV(const std::string& nombreArchivo) {
    std::vector<std::tuple<int, std::vector<int>, double, double>> solicitudes;

    std::ifstream archivo(nombreArchivo);
    if (!archivo.is_open()) {
        std::cerr << "Error al abrir el archivo." << std::endl;
        return solicitudes;
    }

    std::string linea;
    std::getline(archivo, linea); // Ignorar la primera línea (encabezado)

    while (std::getline(archivo, linea)) {
        int usuarioID;
        std::vector<int> microservicios;
        double posX, posY;
        char coma;
        std::istringstream ss(linea);

        ss >> usuarioID >> coma;

        std::string microserviciosStr;
        std::getline(ss, microserviciosStr, ',');
        std::istringstream ssMicro(microserviciosStr);
        std::string micro;
        while (std::getline(ssMicro, micro, ' ')) {
            if (!micro.empty()) {
                try {
                    microservicios.push_back(std::stoi(micro));
                }
                catch (const std::invalid_argument& e) {
                    std::cerr << "Error de conversión al leer microservicio: " << micro << std::endl;
                }
            }
        }

        ss >> posX >> coma >> posY;

        solicitudes.push_back(std::make_tuple(usuarioID, microservicios, posX, posY));
    }

    archivo.close();
    return solicitudes;
}

// Función para leer los drones desde un archivo CSV
std::vector<Drone> leerDronesDesdeCSV(const std::string& nombreArchivo) {
    std::vector<Drone> drones;

    std::ifstream archivo(nombreArchivo);
    if (!archivo.is_open()) {
        std::cerr << "Error al abrir el archivo de drones." << std::endl;
        return drones;
    }

    std::string linea;
    std::getline(archivo, linea); // Ignorar la primera línea (encabezado)

    while (std::getline(archivo, linea)) {
        Drone dron;
        std::istringstream ss(linea);
        ss >> dron.id >> dron.posX >> dron.posY;

        // Leer los microservicios asignados al dron
        int microservicio;
        while (ss >> microservicio) {
            dron.microservicios.push_back(microservicio);
        }

        drones.push_back(dron);
    }

    archivo.close();
    return drones;
}

// Función para escribir los drones en un archivo CSV
void escribirDronesEnCSV(const std::vector<Drone>& drones, const std::string& nombreArchivo) {
    std::ofstream archivo(nombreArchivo);
    if (!archivo.is_open()) {
        std::cerr << "Error al abrir el archivo de salida." << std::endl;
        return;
    }

    archivo << "DronID, CoordenadaX, CoordenadaY, Microservicios\n";
    for (const auto& dron : drones) {
        archivo << dron.id << ", " << dron.posX << ", " << dron.posY << ", ";
        for (int microservicio : dron.microservicios) {
            archivo << microservicio << " ";
        }
        archivo << "\n";
    }

    archivo.close();
}

// Función para leer la asociación de usuarios con el dron más cercano desde un archivo CSV
std::vector<UsuarioDronAsociado> leerAsociacionUsuarioDronCSV(const std::string& nombreArchivo) {
    std::vector<UsuarioDronAsociado> asociaciones;
    std::ifstream archivo(nombreArchivo);

    if (!archivo.is_open()) {
        std::cerr << "Error al abrir el archivo de asociación de usuarios y drones." << std::endl;
        return asociaciones;
    }

    std::string linea;
    std::getline(archivo, linea); // Ignorar la primera línea (encabezado)

    while (std::getline(archivo, linea)) {
        std::istringstream ss(linea);
        int usuarioID, dronID;
        double distancia;
        char coma;

        ss >> usuarioID >> coma >> dronID >> coma >> distancia;
        asociaciones.push_back({usuarioID, dronID, distancia});
    }

    archivo.close();
    return asociaciones;
}

void imprimirMatrizAdyacencia(const std::vector<std::vector<double>>& matriz) {
    int n = matriz.size();
    std::cout << "Matriz de Costes:\n";

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (matriz[i][j] == std::numeric_limits<double>::max()) {
                std::cout << std::setw(5) << "∞" << " ";
            }
            else {
                std::cout << std::setw(5) << matriz[i][j] << " ";
            }
        }
        std::cout << "\n";
    }
}

// Esta función ahora encuentra el camino más corto de vuelta al origen
std::vector<int> encontrarCaminoDeRegreso(const std::vector<std::vector<double>>& matrizAdyacencia, int nodoFinal, int nodoOrigen) {
    auto [distancias, predecesores] = dijkstra(matrizAdyacencia, nodoFinal);
    std::vector<int> camino;

    if (distancias[nodoOrigen] == std::numeric_limits<double>::max()) {
        std::cerr << "No hay camino desde el nodo " << nodoFinal << " de regreso al nodo " << nodoOrigen << std::endl;
        return camino; // Camino vacío si no hay ruta
    }

    for (int at = nodoOrigen; at != -1; at = predecesores[at]) {
        camino.push_back(at);
        if (at == nodoFinal)
            break;
    }
    std::reverse(camino.begin(), camino.end());

    return camino;
}

void imprimirCamino(const std::vector<int>& camino) {
    for (size_t i = 0; i < camino.size(); ++i) {
        std::cout << "Centroide " << camino[i];
        if (i < camino.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
}

double calcularLongitudCamino(const std::vector<int>& camino, const std::vector<Centroide>& centroides) {
    double longitudTotal = 0.0;
    for (size_t i = 1; i < camino.size(); ++i) {
        longitudTotal += calcularDistanciaEnMetros(centroides[camino[i - 1]].posX, centroides[camino[i - 1]].posY, centroides[camino[i]].posX, centroides[camino[i]].posY);
    }
    return longitudTotal;
}

// Definir las especificaciones de los microservicios
std::unordered_map<int, Microservicio> definirMicroservicios() {
    std::unordered_map<int, Microservicio> microservicios;
    microservicios[1] = {"ECG monitoring", 1536, 33.3, 5};    // Valores en MB
    microservicios[2] = {"Compression", 800, 21.5, 1.36};           // Valores en MB
    microservicios[3] = {"Blood pressure monitoring", 1024, 22.8, 6}; // Valores en MB
    microservicios[4] = {"Encryption", 50, 11.4, 0.032};            // Valores en MB
    return microservicios;
}

// Función para encontrar el centroide adecuado para un microservicio específico
Centroide* encontrarCentroideAdecuado(std::vector<Centroide>& centroides, int nodoActual, int microservicio, double inputSize, double ram) {
    double distanciaMinima = std::numeric_limits<double>::max();
    Centroide* centroideAdecuado = nullptr;

    for (auto& centroide : centroides) {
        bool suficienteMemoria = false;

        if (std::find(centroide.microservicios.begin(), centroide.microservicios.end(), microservicio) != centroide.microservicios.end()) {
            if (centroide.microserviciosDesplegados.find(microservicio) == centroide.microserviciosDesplegados.end()) {
                if (centroide.memoriaRestante >= (inputSize + ram)) {
                    suficienteMemoria = true;
                }
            } else {
                if (centroide.memoriaRestante >= inputSize) {
                    suficienteMemoria = true;
                }
            }
        }

        if (suficienteMemoria) {
            double distancia = calcularDistanciaEnMetros(centroides[nodoActual].posX, centroides[nodoActual].posY, centroide.posX, centroide.posY);
            if (distancia < distanciaMinima) {
                distanciaMinima = distancia;
                centroideAdecuado = &centroide;
            }
        }
    }

    return centroideAdecuado;
}

int main() {
    std::vector<Drone> drones = {
        {0, 39.18852, -5.13606, {}},
        {1, 39.18027, -5.10477, {}},
        {2, 39.17741, -5.07588, {}},
        {3, 39.1832, -5.1367, {}},
        {4, 39.16803, -5.10884, {}},
        {5, 39.16823, -5.0455, {}},
        {6, 39.16231, -5.07966, {}},
        {7, 39.16564, -5.05078, {}},
        {8, 39.17133, -5.0549, {}}
    };

    // Asignar microservicios fijos a los drones
    asignarMicroserviciosFijos(drones);

    auto matrizAdyacencia = calcularMatrizAdyacencia(drones);
    imprimirMatrizAdyacencia(matrizAdyacencia);

    escribirDronesEnCSV(drones, "drones.csv");
    std::vector<Drone> dronesLeidos = leerDronesDesdeCSV("drones.csv");

    std::ofstream archivo2("solicitudes.csv");
    archivo2 << "UsuarioID, MicroserviciosRequeridos, PosicionUsuarioX, PosicionUsuarioY\n";
    archivo2 << "0, 3 4, 39.17336, -5.05669\n";
    archivo2 << "1, 1 2 4, 39.16721, -5.05716\n";
    archivo2 << "2, 3 4, 39.17083, -5.05098\n";
    archivo2 << "3, 1 2 4, 39.16757, -5.04703\n";
    archivo2 << "4, 3 4, 39.16691, -5.05231\n";
    archivo2 << "5, 1 2 4, 39.16391, -5.04802\n";
    archivo2 << "6, 3 4, 39.18823, -5.13451\n";
    archivo2 << "7, 1 2 4, 39.1837, -5.12816\n";
    archivo2 << "8, 1 2 4, 39.18616, -5.13417\n";
    archivo2 << "9, 1 2 4, 39.17752, -5.11554\n";
    archivo2 << "10, 3 4, 39.17379, -5.11469\n";
    archivo2 << "11, 1 2 4, 39.17186, -5.09134\n";
    archivo2 << "12, 3 4, 39.17658, -5.11778\n";
    archivo2 << "13, 3 4, 39.17226, -5.10052\n";
    archivo2 << "14, 1 2 4, 39.1676, -5.07709\n";
    archivo2 << "15, 1 2 4, 39.17545, -5.06937\n";
    archivo2 << "16, 3 4, 39.16159, -5.09014\n";
    archivo2 << "17, 3 4, 39.1719, -5.06799\n";
    archivo2 << "18, 1 2 4, 39.17011, -5.054\n";
    archivo2 << "19, 3 4, 39.17783, -5.10576\n"; //19


    archivo2 << "20, 1 2 4, 39.16405, -5.06216\n"; //20
    archivo2 << "21, 1 2 4, 39.16159, -5.09014\n";
    archivo2 << "22, 3 4, 39.14899, -5.07709\n";
    archivo2 << "23, 1 2 4, 39.1643, -5.03125\n";
    archivo2 << "24, 3 4, 39.15884, -5.03211\n";
    archivo2 << "25, 3 4, 39.16523, -5.0137\n";
    archivo2 << "26, 1 2 4, 39.14992, -5.08927\n";
    archivo2 << "27, 3 4, 39.18838, -5.06833\n";
    archivo2 << "28, 1 2 4, 39.18852, -5.08807\n";
    archivo2 << "29, 3 4, 39.18373, -5.07108\n"; //29
    archivo2 << "30, 1 2 4, 39.16177, -5.12618\n"; //30
    archivo2 << "31, 1 2 4, 39.15858, -5.10678\n";
    archivo2 << "32, 1 2 4, 39.16853, -5.14826\n";
    archivo2 << "33, 3 4, 39.18064, -5.13951\n";
    archivo2 << "34, 3 4, 39.17452, -5.13522\n";
    archivo2 << "35, 3 4, 39.15429, -5.12389\n";
    archivo2 << "36, 1 2 4, 39.18796, -5.10724\n";
    archivo2 << "37, 1 2 4, 39.14511, -5.12457\n";
    archivo2 << "38, 3 4, 39.1998, -5.09762\n";
    archivo2 << "39, 1 2 4, 39.19475, -5.14191\n"; //39


    archivo2 << "40, 3 4, 39.15773, -5.05979\n"; //40
    archivo2 << "41, 3 4, 39.14269, -5.07335\n";
    archivo2 << "42, 1 2 4, 39.17197, -5.05739\n";
    archivo2 << "43, 1 2 4, 39.16918, -5.05197\n";
    archivo2 << "44, 3 4, 39.16519, -5.05313\n";
    archivo2 << "45, 1 2 4, 39.16656, -5.0476\n";
    archivo2 << "46, 1 2 4, 39.16785, -5.04004\n";
    archivo2 << "47, 3 4, 39.17111, -5.04605\n";
    archivo2 << "48, 3 4, 39.16456, -5.06407\n";
    archivo2 << "49, 3 4, 39.1613, -5.0539\n";
    archivo2 << "50, 1 2 4, 39.16635, -5.06729\n";
    archivo2 << "51, 1 2 4, 39.18954, -5.13794\n";
    archivo2 << "52, 3 4, 39.19143, -5.13004\n";
    archivo2 << "53, 1 2 4, 39.19711, -5.14793\n";
    archivo2 << "54, 3 4, 39.18734, -5.13845\n";
    archivo2 << "55, 1 2 4, 39.19073, -5.1293\n";
    archivo2 << "56, 3 4, 39.18211, -5.13038\n";
    archivo2 << "57, 1 2 4, 39.19335, -5.13922\n";
    archivo2 << "58, 1 2 4, 39.19348, -5.12339\n";
    archivo2 << "59, 3 4, 39.19096, -5.12214\n"; //59


    archivo2 << "60, 1 2 4, 39.18545, -5.16718\n"; //60
    archivo2 << "61, 1 2 4, 39.19496, -5.14752\n";
    archivo2 << "62, 3 4, 39.17308, -5.1174\n";
    archivo2 << "63, 1 2 4, 39.16036, -5.1198\n";
    archivo2 << "64, 1 2 4, 39.15159, -5.10761\n";
    archivo2 << "65, 3 4, 39.16765, -5.06378\n";
    archivo2 << "66, 3 4, 39.16828, -5.06565\n";
    archivo2 << "67, 3 4, 39.16488, -5.06292\n";
    archivo2 << "68, 1 2 4, 39.16841, -5.05579\n";
    archivo2 << "69, 1 2 4, 39.16932, -5.05595\n";
    archivo2 << "70, 3 4, 39.1688, -5.06184\n";
    archivo2 << "71, 1 2 4, 39.16909, -5.06473\n";
    archivo2 << "72, 3 4, 39.16404, -5.06505\n";
    archivo2 << "73, 1 2 4, 39.1703, -5.05568\n";
    archivo2 << "74, 3 4, 39.1557, -5.04405\n";
    archivo2 << "75, 1 2 4, 39.16766, -5.09129\n";
    archivo2 << "76, 3 4, 39.17087, -5.06061\n";
    archivo2 << "77, 1 2 4, 39.17592, -5.04687\n";
    archivo2 << "78, 3 4, 39.16894, -5.10962\n";
    archivo2 << "79, 1 2 4, 39.16812, -5.11025\n"; //79


    /*archivo2 << "80, 3 4, 39.19899, -5.16674\n"; //80
    archivo2 << "81, 3 4, 39.19317, -5.15936\n";
    archivo2 << "82, 1 2 4, 39.20168, -5.14267\n";
    archivo2 << "83, 3 4, 39.18698, -5.17198\n";
    archivo2 << "84, 1 2 4, 39.19672, -5.1676\n";
    archivo2 << "85, 3 4, 39.17965, -5.11929\n";
    archivo2 << "86, 1 2 4, 39.18607, -5.13207\n";
    archivo2 << "87, 3 4, 39.17803, -5.13375\n";
    archivo2 << "88, 1 2 4, 39.1888, -5.13416\n";
    archivo2 << "89, 1 2 4, 39.19142, -5.13096\n";
    archivo2 << "90, 3 4, 39.19101, -5.13375\n";
    archivo2 << "91, 3 4, 39.18944, -5.13412\n";
    archivo2 << "92, 1 2 4, 39.18964, -5.13666\n";
    archivo2 << "93, 3 4, 39.17937, -5.10833\n";
    archivo2 << "94, 3 4, 39.18483, -5.09562\n";
    archivo2 << "95, 3 4, 39.19086, -5.09741\n";
    archivo2 << "96, 1 2 4, 39.1811, -5.08738\n";
    archivo2 << "97, 3 4, 39.16844, -5.04873\n";
    archivo2 << "90, 1 2 4, 39.17101, -5.04929\n";
    archivo2 << "99, 3 4, 39.17715, -5.08249\n"; //99*/

    archivo2.close();

    std::vector<std::tuple<int, std::vector<int>, double, double>> solicitudes = leerArchivoCSV("solicitudes.csv");
    std::vector<UsuarioDronAsociado> asociaciones = leerAsociacionUsuarioDronCSV("usuarios_y_drones80.csv");

    auto microservicios = definirMicroservicios();

    // Aplicar k-means a los drones usando los centroides 0, 2, 4 y 8
    int k = 4; // Número de clusters deseado
    std::vector<int> centroidIndices = {0, 2, 4, 8};
    auto [centroidesDrones, asignaciones] = kMeansClustering(drones, k, centroidIndices);

    // Crear centroides con memoria inicial y microservicios
    std::vector<Centroide> centroides(k);
    for (int i = 0; i < k; ++i) {
        centroides[i] = Centroide(centroidesDrones[i].posX, centroidesDrones[i].posY);
    }

    // Asignar microservicios a los centroides según las nuevas especificaciones
    centroides[0].microservicios = {1, 2, 3, 4};
    centroides[1].microservicios = {1, 2, 4};
    centroides[2].microservicios = {1, 3};
    centroides[3].microservicios = {1, 2, 3, 4};

    //Replicacion del 25%
    /*centroides[0].microservicios = {1};   // Microservicio 1 replicado en el centorid 0
    centroides[1].microservicios = {2};   // Microservicio 2 replicado en el centorid 1
    centroides[2].microservicios = {3};   // Microservicio 3 replicado en el centorid 2
    centroides[3].microservicios = {4};   // Microservicio 4 replicado en el centorid 3*/

    //Replicacion del 50%
    /*centroides[0].microservicios = {1, 2};   // Microservicios 1 y 2 replicados en el centorid 0
    centroides[1].microservicios = {2, 3};   // Microservicios 2 y 3 replicados en el centorid 1
    centroides[2].microservicios = {3, 4};   // Microservicios 3 y 4 replicados en el centorid 2
    centroides[3].microservicios = {1, 4};   // Microservicios 1 y 4 replicados en el centorid 3*/


    //Replicacion del 100%
    /*centroides[0].microservicios = {1, 2, 3, 4};   // Microservicios 1, 2, 3, 4 replicados en el centorid 0
    centroides[1].microservicios = {1, 2, 3, 4};   // Microservicios 1, 2, 3, 4 replicados en el centorid 1
    centroides[2].microservicios = {1, 2, 3, 4};   // Microservicios 1, 2, 3, 4 replicados en el centorid 2
    centroides[3].microservicios = {1, 2, 3, 4};   // Microservicios 1, 2, 3, 4 replicados en el centorid 3*/


    // Mostrar los centroides y las asignaciones
    std::cout << "Centroides después de aplicar k-means:\n";
    for (int i = 0; i < k; ++i) {
        std::cout << "Centroide " << i << ": (" << centroides[i].posX << ", " << centroides[i].posY << ")\n";
        std::cout << "Microservicios del centroide " << i << ": ";
        for (int microservicio : centroides[i].microservicios) {
            std::cout << microservicio << " ";
        }
        std::cout << "\n";
    }

    std::cout << "\nAsignaciones de drones a clusters:\n";
    for (int i = 0; i < drones.size(); ++i) {
        std::cout << "Dron " << drones[i].id << " asignado a cluster " << asignaciones[i] << "\n";
    }

    std::ofstream archivoLongitudes("longitudesKMeans.csv");

    for (const auto& solicitud : solicitudes) {
        int usuarioID;
        std::vector<int> microserviciosRequeridos;
        double posX, posY;
        std::tie(usuarioID, microserviciosRequeridos, posX, posY) = solicitud;

        auto it = std::find_if(asociaciones.begin(), asociaciones.end(), [usuarioID](const UsuarioDronAsociado& uda) {
            return uda.usuarioID == usuarioID;
        });

        if (it != asociaciones.end()) {
            if (it->dronID < 0 || it->dronID >= drones.size()) {
                std::cerr << "Dron ID inválido para el usuario " << usuarioID << std::endl;
                continue;
            }

            int nodoOrigen = it->dronID;
            int clusterOrigen = asignaciones[nodoOrigen];
            std::vector<int> todosLosCaminos;
            std::vector<int> caminoRegreso;
            int nodoActual = clusterOrigen;
            bool puedeSatisfacer = true; // Variable para comprobar si se puede satisfacer toda la solicitud

            std::cout << "Para el usuario " << usuarioID << ", el nodo origen es el dron " << nodoOrigen << " en el centroide " << clusterOrigen << std::endl;

            for (int microservicio : microserviciosRequeridos) {
                bool satisfecho = false; // Variable para comprobar si el microservicio específico se ha satisfecho
                while (!satisfecho) {
                    Centroide* centroideSiguiente = encontrarCentroideAdecuado(centroides, nodoActual, microservicio, microservicios[microservicio].inputSize, microservicios[microservicio].ram);
                    if (centroideSiguiente == nullptr) {
                        std::cerr << "No se puede satisfacer la petición para el usuario " << usuarioID << " porque no puede abastecerse del microservicio " << microservicio << std::endl;
                        puedeSatisfacer = false;
                        break;
                    }

                    int idCentroideSiguiente = std::distance(centroides.begin(), std::find(centroides.begin(), centroides.end(), *centroideSiguiente));
                    todosLosCaminos.push_back(idCentroideSiguiente);

                    double inputSize = microservicios[microservicio].inputSize;
                    double ram = microservicios[microservicio].ram;

                    if (centroideSiguiente->microserviciosDesplegados.find(microservicio) == centroideSiguiente->microserviciosDesplegados.end()) {
                        if (centroideSiguiente->memoriaRestante >= (inputSize + ram)) {
                            centroideSiguiente->memoriaRestante -= ram;
                            centroideSiguiente->microserviciosDesplegados[microservicio] = true;
                            std::cout << "Desplegando microservicio " << microservicio << " en centroide " 
                                      << idCentroideSiguiente << ". RAM utilizada: " << ram << " MB, memoria restante: " 
                                      << centroideSiguiente->memoriaRestante << " MB" << std::endl;
                        } else {
                            nodoActual = idCentroideSiguiente;
                            continue; // Busca otro centroide con suficiente memoria
                        }
                    }

                    if (centroideSiguiente->memoriaRestante >= inputSize) {
                        centroideSiguiente->memoriaRestante -= inputSize;
                        std::cout << "Accediendo a microservicio " << microservicio << " en centroide " 
                                  << idCentroideSiguiente << ". Tamaño de entrada: " << inputSize << " MB, memoria restante: " 
                                  << centroideSiguiente->memoriaRestante << " MB" << std::endl;
                        nodoActual = idCentroideSiguiente; // actualizar el nodo actual
                        satisfecho = true;
                    } else {
                        nodoActual = idCentroideSiguiente;
                    }
                }
                if (!puedeSatisfacer) break;
            }

            if (puedeSatisfacer) {
                // Añadir la lógica para regresar al nodo origen por el camino más corto
                if (nodoActual != clusterOrigen) {
                    caminoRegreso = encontrarCaminoDeRegreso(matrizAdyacencia, nodoActual, clusterOrigen);
                }

                double longitudInicial = 0.0;
                if (!todosLosCaminos.empty()) {
                    longitudInicial = calcularDistanciaEnMetros(centroides[clusterOrigen].posX, centroides[clusterOrigen].posY, centroides[todosLosCaminos[0]].posX, centroides[todosLosCaminos[0]].posY);
                }
                double longitudCaminoBusqueda = calcularLongitudCamino(todosLosCaminos, centroides);
                double longitudCaminoRegreso = calcularLongitudCamino(caminoRegreso, centroides);
                double longitudTotal = longitudInicial + longitudCaminoBusqueda + longitudCaminoRegreso;

                std::cout << "El camino de búsqueda de microservicios para el usuario " << usuarioID << " es: ";
                imprimirCamino(todosLosCaminos);

                std::cout << "El camino de regreso para el usuario " << usuarioID << " es: ";
                imprimirCamino(caminoRegreso);

                std::cout << "La longitud total del camino recorrido para el usuario " << usuarioID << " es: " << longitudTotal << " metros." << std::endl;

                //Escribir la longitud en el archivo CSV
                archivoLongitudes << longitudTotal << "\n";
            }
        }
        else {
            std::cerr << "No se encontró asociación para el usuario " << usuarioID << std::endl;
        }
    }

    return 0;
}
