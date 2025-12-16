//Modulo:database.cpp
//Autor: Erwan Bernard
//Fecha:15/03/25
//Declaracion de las funciones del modulo database.hpp

#include "database.hpp"


Database::Database(const std::string& ficheroDb){
  const char* FILE_DB=ficheroDb.c_str();
  //Abriendo la base de datos
  
  if (sqlite3_open(FILE_DB,&m_db)!=SQLITE_OK){
    std::cout<<"Error abriendo la base de datos"<<std::endl;
    sqlite3_close(m_db);
  };

}
Database::~Database(){
  sqlite3_close(m_db);
}

sqlite3* Database::getDb(){
  return m_db;
};

void Database::execute(const std::string& sql){
  const char* SQL_INSTRUCCION=sql.c_str();
  sqlite3_exec(m_db,SQL_INSTRUCCION, nullptr,nullptr,nullptr);
}
