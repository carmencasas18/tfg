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
    drones[9].microservicios = {2, 4};
    drones[10].microservicios = {1, 4};
    drones[11].microservicios = {2, 3};
    drones[12].microservicios = {1, 2};
    drones[13].microservicios = {3, 4};
    drones[14].microservicios = {1, 3};
    drones[15].microservicios = {2, 4};
    drones[16].microservicios = {1, 4};
    drones[17].microservicios = {2, 3};
    drones[18].microservicios = {1, 2};
    drones[19].microservicios = {3, 4};
    drones[20].microservicios = {1, 3};
    drones[21].microservicios = {2, 4};
    drones[22].microservicios = {1, 4};
    drones[23].microservicios = {2, 3};
    drones[24].microservicios = {1, 2};
    drones[25].microservicios = {3, 4};
    drones[26].microservicios = {1, 3};
    drones[27].microservicios = {2, 4};
    drones[28].microservicios = {1, 4};
    drones[29].microservicios = {2, 3};
    drones[30].microservicios = {1, 2};
    drones[31].microservicios = {3, 4};
    drones[32].microservicios = {1, 3};
    drones[33].microservicios = {2, 4};
    drones[34].microservicios = {1, 4};
    drones[35].microservicios = {2, 3};
    drones[36].microservicios = {1, 2};
    drones[37].microservicios = {3, 4};
    drones[38].microservicios = {1, 3};
    drones[39].microservicios = {2, 4};
    drones[40].microservicios = {1, 4};
    drones[41].microservicios = {2, 3};
    drones[42].microservicios = {1, 2};
    drones[43].microservicios = {3, 4};
    drones[44].microservicios = {1, 3};
    drones[45].microservicios = {2, 4};
    drones[46].microservicios = {1, 4};
    drones[47].microservicios = {2, 3};
    drones[48].microservicios = {1, 2};
    drones[49].microservicios = {3, 4};
    drones[50].microservicios = {1, 3};
    drones[51].microservicios = {2, 4};
    drones[52].microservicios = {1, 4};
    drones[53].microservicios = {2, 3};
    drones[54].microservicios = {1, 2};
    drones[55].microservicios = {3, 4};
    drones[56].microservicios = {1, 3};
    drones[57].microservicios = {2, 4};
    drones[58].microservicios = {1, 4};
    drones[59].microservicios = {2, 3};
    /*drones[60].microservicios = {1, 2};
    drones[61].microservicios = {3, 4};
    drones[62].microservicios = {1, 3};
    drones[63].microservicios = {2, 4};
    drones[64].microservicios = {1, 4};
    drones[65].microservicios = {2, 3};
    drones[66].microservicios = {1, 2};
    drones[67].microservicios = {3, 4};
    drones[68].microservicios = {1, 3};
    drones[69].microservicios = {2, 4};
    drones[70].microservicios = {1, 4};
    drones[71].microservicios = {2, 3};
    drones[72].microservicios = {1, 2};
    drones[73].microservicios = {3, 4};
    drones[74].microservicios = {1, 3};
    drones[75].microservicios = {2, 4};
    drones[76].microservicios = {1, 4};
    drones[77].microservicios = {2, 3};
    drones[78].microservicios = {1, 2};
    drones[79].microservicios = {3, 4};
    drones[80].microservicios = {1, 3};
    drones[81].microservicios = {2, 4};
    drones[82].microservicios = {1, 4};
    drones[83].microservicios = {2, 3};
    drones[84].microservicios = {1, 2};
    drones[85].microservicios = {3, 4};
    drones[86].microservicios = {1, 3};
    drones[87].microservicios = {2, 4};
    drones[88].microservicios = {1, 4};
    drones[89].microservicios = {2, 3};
    drones[90].microservicios = {1, 2};
    drones[91].microservicios = {3, 4};
    drones[92].microservicios = {1, 3};
    drones[93].microservicios = {2, 4};
    drones[94].microservicios = {1, 4};
    drones[95].microservicios = {2, 3};
    drones[96].microservicios = {1, 2};
    drones[97].microservicios = {3, 4};
    drones[98].microservicios = {1, 3};
    drones[99].microservicios = {2, 4};*/
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
        {0, 39.140541, -5.044044, {}},
        {1, 39.139341, -5.062368, {}},
        {2, 39.146132, -5.035888, {}},
        {3, 39.123670, -5.074702, {}},
        {4, 39.153288, -5.055550, {}},
        {5, 39.163104, -5.052651, {}},
        {6, 39.130504, -5.071467, {}},
        {7, 39.144213, -5.043655, {}},
        {8, 39.144052, -5.061345, {}},
        {9, 39.127404, -5.075540, {}},
        {10, 39.136716, -5.037387, {}},
        {11, 39.148865, -5.063997, {}},
        {12, 39.161147, -5.064975, {}},
        {13, 39.152483, -5.043278, {}},
        {14, 39.126946, -5.061464, {}},
        {15, 39.131331, -5.047550, {}},
        {16, 39.129892, -5.070432, {}},
        {17, 39.137108, -5.063126, {}},
        {18, 39.137102, -5.073709, {}},
        {19, 39.164013, -5.064702, {}},
        {20, 39.420234, -5.198911, {}},
        {21, 39.424938, -5.230796, {}},
        {22, 39.427651, -5.197333, {}},
        {23, 39.401693, -5.224458, {}},
        {24, 39.430736, -5.198011, {}},
        {25, 39.418091, -5.195653, {}},
        {26, 39.384337, -5.207041, {}},
        {27, 39.392544, -5.223982, {}},
        {28, 39.430485, -5.226711, {}},
        {29, 39.414754, -5.231186, {}},
        {30, 39.402357, -5.188906, {}},
        {31, 39.420460, -5.185099, {}},
        {32, 39.396958, -5.196021, {}},
        {33, 39.423547, -5.208701, {}},
        {34, 39.429083, -5.211725, {}},
        {35, 39.409330, -5.202039, {}},
        {36, 39.392554, -5.210346, {}},
        {37, 39.406339, -5.220323, {}},
        {38, 39.407248, -5.209131, {}},
        {39, 39.399151, -5.201758, {}},
        {40, 39.186421, -4.931518, {}},
        {41, 39.187478, -4.901687, {}},
        {42, 39.213956, -4.897520, {}},
        {43, 39.211637, -4.932016, {}},
        {44, 39.197570, -4.921276, {}},
        {45, 39.190682, -4.903182, {}},
        {46, 39.232220, -4.910652, {}},
        {47, 39.200645, -4.930468, {}},
        {48, 39.203775, -4.921350, {}},
        {49, 39.220007, -4.890005, {}},
        {50, 39.220549, -4.925176, {}},
        {51, 39.185061, -4.898217, {}},
        {52, 39.197318, -4.929038, {}},
        {53, 39.227316, -4.892341, {}},
        {54, 39.188118, -4.892318, {}},
        {55, 39.207323, -4.904825, {}},
        {56, 39.200970, -4.922033, {}},
        {57, 39.225550, -4.927935, {}},
        {58, 39.226638, -4.886361, {}},
        {59, 39.195516, -4.904895, {}},
        /*{60, 39.27584, -5.08793, {}},
        {61, 39.26717, -5.08141, {}},
        {62, 39.24776, -5.07420, {}},
        {63, 39.23474, -5.07214, {}},
        {64, 39.22729, -5.07214, {}},
        {65, 39.21692, -5.07008, {}},
        {66, 39.20282, -5.06459, {}},
        {67, 39.1975, -5.05326, {}},
        {68, 39.18792, -5.0615, {}},
        {69, 39.17912, -5.05534, {}},
        {70, 39.16678, -5.04084, {}},
        {71, 39.16571, -5.0314, {}},
        {72, 39.16385, -5.02024, {}},
        {73, 39.16079, -5.00977, {}},
        {74, 39.15826, -5.00102, {}},
        {75, 39.15134, -4.9902, {}},
        {76, 39.14641, -4.98196, {}},
        {77, 39.14063, -4.97461, {}},
        {78, 39.13251, -4.96225, {}},
        {79, 39.12904, -4.95349, {}},
        {80, 39.13916, -4.94471, {}},
        {81, 39.14182, -4.95124, {}},
        {82, 39.14751, -4.9552, {}},
        {83, 39.15417, -4.96052, {}},
        {84, 39.15923, -4.96619, {}},
        {85, 39.16508, -4.97151, {}},
        {86, 39.17001, -4.98129, {}},
        {87, 39.17413, -4.98902, {}},
        {88, 39.17932, -4.99709, {}},
        {89, 39.18784, -5.00653, {}},
        {90, 39.20008, -5.01592, {}},
        {91, 39.21631, -5.02622, {}},
        {92, 39.22775, -5.03137, {}},
        {93, 39.24078, -5.03858, {}},
        {94, 39.25487, -5.04442, {}},
        {95, 39.26736, -5.04957, {}},
        {96, 39.27773, -5.05678, {}},
        {97, 39.28809, -5.06605, {}},
        {98, 39.29739, -5.07772, {}},
        {99, 39.21498, -5.06518, {}}*/
    };

    // Asignar microservicios fijos a los drones
    asignarMicroserviciosFijos(drones);

    auto matrizAdyacencia = calcularMatrizAdyacencia(drones);
    imprimirMatrizAdyacencia(matrizAdyacencia);

    escribirDronesEnCSV(drones, "drones.csv");
    std::vector<Drone> dronesLeidos = leerDronesDesdeCSV("drones.csv");

    std::ofstream archivo2("solicitudes.csv");
    archivo2 << "UsuarioID, MicroserviciosRequeridos, PosicionUsuarioX, PosicionUsuarioY\n";
    archivo2 << "0, 3 4, 39.145284, -5.082735\n";
    archivo2 << "1, 1 2 4, 39.147694, -5.041173\n";
    archivo2 << "2, 3 4, 39.157410, -5.040255\n";
    archivo2 << "3, 1 2 4, 39.165178, -5.043864\n";
    archivo2 << "4, 3 4, 39.148741, -5.060922\n";
    archivo2 << "5, 1 2 4, 39.128975, -5.082887\n";
    archivo2 << "6, 3 4, 39.152454, -5.042447\n";
    archivo2 << "7, 1 2 4, 39.152692, -5.047987\n";
    archivo2 << "8, 1 2 4, 39.125786, -5.057714\n";
    archivo2 << "9, 1 2 4, 39.143528, -5.033393\n";
    archivo2 << "10, 3 4, 39.118880, -5.034640\n";
    archivo2 << "11, 1 2 4, 39.128129, -5.068813\n";
    archivo2 << "12, 3 4, 39.116778, -5.072828\n";
    archivo2 << "13, 3 4, 39.155719, -5.045047\n";
    archivo2 << "14, 1 2 4, 39.162496, -5.048641\n";
    archivo2 << "15, 1 2 4, 39.139231, -5.048010\n";
    archivo2 << "16, 3 4, 39.119166, -5.033596\n";
    archivo2 << "17, 3 4, 39.155020, -5.048181\n";
    archivo2 << "18, 1 2 4, 39.129395, -5.044686\n";
    archivo2 << "19, 3 4, 39.131838, -5.053373\n"; //19


    archivo2 << "20, 1 2 4, 39.118675, -5.038111\n"; //20
    archivo2 << "21, 1 2 4, 39.126965, -5.053252\n";
    archivo2 << "22, 3 4, 39.125882, -5.044532\n";
    archivo2 << "23, 1 2 4, 39.147985, -5.075836\n";
    archivo2 << "24, 3 4, 39.122278, -5.053203\n";
    archivo2 << "25, 3 4, 39.149874, -5.049951\n";
    archivo2 << "26, 1 2 4, 39.144739, -5.053007\n";
    archivo2 << "27, 3 4, 39.135085, -5.040396\n";
    archivo2 << "28, 1 2 4, 39.124860, -5.046061\n";
    archivo2 << "29, 3 4, 39.117331, -5.043570\n"; //29
    archivo2 << "30, 1 2 4, 39.162993, -5.038542\n"; //30
    archivo2 << "31, 1 2 4, 39.118390, -5.076440\n";
    archivo2 << "32, 1 2 4, 39.118732, -5.035855\n";
    archivo2 << "33, 3 4, 39.127131, -5.062301\n";
    archivo2 << "34, 3 4, 39.432688, -5.215527\n";
    archivo2 << "35, 3 4, 39.390797, -5.214538\n";
    archivo2 << "36, 1 2 4, 39.410609, -5.229535\n";
    archivo2 << "37, 1 2 4, 39.397429, -5.225117\n";
    archivo2 << "38, 3 4, 39.392849, -5.188891\n";
    archivo2 << "39, 1 2 4, 39.419625, -5.205497\n"; //39


    archivo2 << "40, 3 4, 39.389287, -5.202013\n"; //40
    archivo2 << "41, 3 4, 39.431475, -5.214847\n";
    archivo2 << "42, 1 2 4, 39.398314, -5.193247\n";
    archivo2 << "43, 1 2 4, 39.410447, -5.221940\n";
    archivo2 << "44, 3 4, 39.393781, -5.217503\n";
    archivo2 << "45, 1 2 4, 39.414430, -5.231126\n";
    archivo2 << "46, 1 2 4, 39.384065, -5.204740\n";
    archivo2 << "47, 3 4, 39.388440, -5.205544\n";
    archivo2 << "48, 3 4, 39.429173, -5.204758\n";
    archivo2 << "49, 3 4, 39.405096, -5.228588\n";
    archivo2 << "50, 1 2 4, 39.389928, -5.188768\n";
    archivo2 << "51, 1 2 4, 39.389084, -5.221439\n";
    archivo2 << "52, 3 4, 39.388393, -5.231492\n";
    archivo2 << "53, 1 2 4, 39.429719, -5.200666\n";
    archivo2 << "54, 3 4, 39.429655, -5.187102\n";
    archivo2 << "55, 1 2 4, 39.393784, -5.211185\n";
    archivo2 << "56, 3 4, 39.414712, -5.202145\n";
    archivo2 << "57, 1 2 4, 39.416125, -5.198602\n";
    archivo2 << "58, 1 2 4, 39.414737, -5.230353\n";
    archivo2 << "59, 3 4, 39.394163, -5.198259\n"; //59


    archivo2 << "60, 1 2 4, 39.431515, -5.191413\n"; //60
    archivo2 << "61, 1 2 4, 39.409733, -5.192191\n";
    archivo2 << "62, 3 4, 39.418183, -5.232095\n";
    archivo2 << "63, 1 2 4, 39.389803, -5.210852\n";
    archivo2 << "64, 1 2 4, 39.386862, -5.207186\n";
    archivo2 << "65, 3 4, 39.208247, -4.885061\n";
    archivo2 << "66, 3 4, 39.203836, -4.912551\n";
    archivo2 << "67, 3 4, 39.187980, -4.922397\n";
    archivo2 << "68, 1 2 4, 39.214411, -4.884092\n";
    archivo2 << "69, 1 2 4, 39.220385, -4.928119\n";
    archivo2 << "70, 3 4, 39.209339, -4.909622\n";
    archivo2 << "71, 1 2 4, 39.223938, -4.892694\n";
    archivo2 << "72, 3 4, 39.188794, -4.890029\n";
    archivo2 << "73, 1 2 4, 39.221112, -4.899044\n";
    archivo2 << "74, 3 4, 39.225846, -4.925736\n";
    archivo2 << "75, 1 2 4, 39.194625, -4.932040\n";
    archivo2 << "76, 3 4, 39.206623, -4.921941\n";
    archivo2 << "77, 1 2 4, 39.184278, -4.907953\n";
    archivo2 << "78, 3 4, 39.210162, -4.926747\n";
    archivo2 << "79, 1 2 4, 39.211775, -4.931844\n"; //79


    archivo2 << "80, 3 4, 39.216964, -4.922100\n"; //80
    archivo2 << "81, 3 4, 39.197161, -4.890767\n";
    archivo2 << "82, 1 2 4, 39.205137, -4.890356\n";
    archivo2 << "83, 3 4, 39.224592, -4.913807\n";
    archivo2 << "84, 1 2 4, 39.212080, -4.903115\n";
    archivo2 << "85, 3 4, 39.201936, -4.886510\n";
    archivo2 << "86, 1 2 4, 39.198182, -4.910077\n";
    archivo2 << "87, 3 4, 39.232339, -4.890244\n";
    archivo2 << "88, 1 2 4, 39.216247, -4.899291\n";
    archivo2 << "89, 1 2 4, 39.216654, -4.894588\n";
    archivo2 << "90, 3 4, 39.201200, -4.907678\n";
    archivo2 << "91, 3 4, 39.211238, -4.897873\n";
    archivo2 << "92, 1 2 4, 39.225276, -4.926285\n";
    archivo2 << "93, 3 4, 39.216175, -4.895373\n";
    archivo2 << "94, 3 4, 39.224261, -4.910458\n";
    archivo2 << "95, 3 4, 39.189494, -4.889305\n";
    archivo2 << "96, 1 2 4, 39.221021, -4.911562\n";
    archivo2 << "97, 3 4, 39.184620, -4.894810\n";
    archivo2 << "98, 1 2 4, 39.215312, -4.907130\n";
    archivo2 << "99, 3 4, 39.201321, -4.909876\n"; //99

    archivo2.close();

    std::vector<std::tuple<int, std::vector<int>, double, double>> solicitudes = leerArchivoCSV("solicitudes.csv");
    std::vector<UsuarioDronAsociado> asociaciones = leerAsociacionUsuarioDronCSV("prueba100.csv");

    auto microservicios = definirMicroservicios();

    // Aplicar k-means a los drones usando los centroides 0, 2, 4 y 8
    int k = 15; // Número de clusters deseado
    std::vector<int> centroidIndices = {0, 4, 8, 10, 18,
                                        20, 24, 28, 32, 36,
                                        40, 44, 48, 52, 56,};
    auto [centroidesDrones, asignaciones] = kMeansClustering(drones, k, centroidIndices);

    // Crear centroides con memoria inicial y microservicios
    std::vector<Centroide> centroides(k);
    for (int i = 0; i < k; ++i) {
        centroides[i] = Centroide(centroidesDrones[i].posX, centroidesDrones[i].posY);
    }

    // Asignar microservicios a los centroides según las nuevas especificaciones
    centroides[0].microservicios = {1, 2, 3, 4};
    centroides[1].microservicios = {1, 2, 3, 4};
    centroides[2].microservicios = {1, 3};
    centroides[3].microservicios = {1, 2, 3, 4};
    centroides[4].microservicios = {2, 4};

    centroides[5].microservicios = {1, 2, 3, 4};
    centroides[6].microservicios = {2, 4};
    centroides[7].microservicios = {1, 3};
    centroides[8].microservicios = {1, 2, 3, 4};
    centroides[9].microservicios = {1, 2, 3, 4};

    centroides[10].microservicios = {1, 3};
    centroides[11].microservicios = {1, 2, 3, 4};
    centroides[12].microservicios = {1, 2, 3, 4};
    centroides[13].microservicios = {1, 2, 3, 4};
    centroides[14].microservicios = {1, 3};

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

    int solicitudesCompletadas = 0;

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

                // Incrementar el contador de solicitudes completadas
                solicitudesCompletadas++;
            }
        }
        else {
            std::cerr << "No se encontró asociación para el usuario " << usuarioID << std::endl;
        }
    }

    archivoLongitudes.close();

    // Calcular y mostrar el porcentaje de solicitudes completadas
    double porcentajeCompletadas = (static_cast<double>(solicitudesCompletadas) / solicitudes.size()) * 100;
    std::cout << "Porcentaje de solicitudes completadas: " << porcentajeCompletadas << "%" << std::endl;

    return 0;
}
