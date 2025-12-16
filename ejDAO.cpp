//  Programa: ejDAO.cpp
// Propósito: Usar clases y sqlite usando DAO (Data Access Object)
//            y DTO (Data Transfer Object)
//     Autor: Javier Macías
//     Fecha: 15/03/2025
//      Obs.: Se usa la base de datos de países y personas tal y
//            como quedó, aunque para facilitar la programación
//            se puede ejecutar antes en sqlite3:
//              UPDATE persona SET dniNie = ''
//              WHERE dinNie is null;

#include "database.hpp"
#include "dto.hpp"
#include "dao.hpp"
#include <string>
#include <iostream>
#include <utility>

const std::string FI_BD = "pruebas.db";

int main()
{
  Database database(FI_BD);
  std::string codPais;
  int idPersona;
 
  //Preguntamos por un país
  std::cout << "Dime un código de país: ";
  std::cin >> codPais;

  //Buscamos y mostramos el país
  PaisDAO paisDAO(database);  
  PaisDTO pais = paisDAO.getPaisPorCodigo(codPais);
  if (!pais.esNulo()) {
    std::cout << " " << codPais << " es " << pais.getNombre() << std::endl;
  } else {
    std::cout << " No encontrado el código de país " << codPais << std::endl;
  }

  //Preguntamos por una persona
  std::cout << std::endl << "Dime un identificador de persona: ";
  std::cin >> idPersona;

  //Buscamos y mostramos la persona
  PersonaDAO personaDAO(database);
  t_personaPais personaPais = personaDAO.getPersonaExtPorId(idPersona);
  if (!personaPais.persona.esNulo()) {
    std::cout << " La persona con identificador " << idPersona 
              << " se llama " << personaPais.persona.getNombre()
              << " y es de " << personaPais.pais.getNombre() << std::endl;
  } else {
    std::cout << " No encontrada la persona con identificador " << idPersona << std::endl;  
  }

}
