//    Módulo: database.hpp
// Propósito: Declaración clase Database, que conecta y desconecta
//            una base de datos sqlite y permite ejecutar SQL que 
//            no devuelven resultados.
//     Autor: Javier Macías
//     Fecha: 14/03/2025
//      Obs.: En caso de detectar un error, lanza un throw

#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <sqlite3.h>
#include <string>
#include <iostream>

class Database
{
  public:
    Database(const std::string& ficheroDb);
    ~Database();
    void execute(const std::string& sql);
    sqlite3* getDb();

  private:
    sqlite3* m_db;
};

#endif  //DATABASE_HPP
