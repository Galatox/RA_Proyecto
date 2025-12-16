//    Módulo: dao.hpp
// Propósito: Declaración de las clases DAO (Data Access Object),
//            que independizan las clases del "negocio" del acceso
//            a las bases de datos (o similares)
//     Autor: Javier Macías
//     Fecha: 14/03/2025
//      Obs.: La tabla pais debe crearse antes que la tabla persona
//            porque persona referencia a pais.

#ifndef DAO_HPP
#define DAO_HPP

#include "dto.hpp"
#include "database.hpp"
#include <string>
#include <utility>

typedef struct
{
  PersonaDTO persona;
  PaisDTO pais;
} t_personaPais;  //para devolver los datos de una persona extendidos con los de su país

class PaisDAO
{
  public:
    PaisDAO(Database& database);
    void crearTablaPais();  //crea, si no existe, la tabla pais;
    PaisDTO getPaisPorCodigo(std::string codigo);  //si no existe, devuelve país vacío
  private:
    Database m_database;
};

class PersonaDAO
{
  public:
    PersonaDAO(Database& database);
    void crearTablaPersona();  //crea, si no existe, la tabla persona;
    PersonaDTO getPersonaPorId(int id);
    t_personaPais getPersonaExtPorId(int id);
  private:
    Database m_database;
};

#endif   //DAO_HPP
