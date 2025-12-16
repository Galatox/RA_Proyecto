#include "dao.hpp"
#include <iostream>
#include <stdexcept>


//PaisDAO

PaisDAO::PaisDAO(Database& database):m_database(database){
};

void PaisDAO::crearTablaPais(){
    const std::string& SQL_TABLA="CREATE TABLE IF NOT EXISTS pais("
                          "codigo TEXT PRIMARY KEY,"
                          "nombre TEXT NOT NULL,"
                          "telf INT,"
                          "contacto TEXT" 
                            ");";
   m_database.execute(SQL_TABLA);
}

PaisDTO PaisDAO::getPaisPorCodigo(std::string codigo){
    //Declaracion de la SQL
    std::string SQL_BUSCAPAIS="SELECT nombre FROM pais "
                              "WHERE codigo='";
    std::string finalizar="';";
    const char* SQL_PAIS_COD=(SQL_BUSCAPAIS+codigo+finalizar).c_str();
    
    //Prepararando la consulta
    sqlite3_stmt* stmt;
    int rc;
    if((sqlite3_prepare_v2(m_database.getDb(),SQL_PAIS_COD,-1,&stmt,nullptr))!=SQLITE_OK){
        throw std::runtime_error("Error, en la preparacion de las consultas pais");
    }
    rc=sqlite3_step(stmt);
    if(rc==SQLITE_ROW){
     std::string  nomPais=reinterpret_cast<const char*>(sqlite3_column_text(stmt,0));
     PaisDTO pais(codigo,nomPais,0,"");
     pais.getNombre();
    };
    
    if(rc==SQLITE_DONE){
        PaisDTO nulo;
        nulo.esNulo();
    };

    sqlite3_finalize(stmt);
}

//Clase PersonaDAO

PersonaDAO::PersonaDAO(Database& database):m_database(database){
}

void PersonaDAO::crearTablaPersona(){
    const char* SQL_PERSONA_TABLA="CREATE TABLE IF NOT EXISTS persona( " 
                            "id INTEGER PRIMARY KEY,"
                            "nombre TEXT NOT NULL,"
                            "sexo TEXT CHECK(sexo IN('H','M')) DEFAULT 'No especificado'"
                            "dniNie TEXT,"
                            "fnac TEXT,"
                            "altura INT DEFAULT=0,"
                            "pais TEXT"
                            "FOREIGN KEY pais REFERENCES pais(codigo)"
                            ");"
                            "PRAGMA foreign_keys=on";
    m_database.execute(SQL_PERSONA_TABLA);                  
}

PersonaDTO PersonaDAO::getPersonaPorId(int id){
    
    std::string SQL_PERSONA="SELECT nombre, pais FROM persona, "
                            "WHERE id=";
    std::string finalizar=  ";";
    std::string idString=std::to_string(id);
    const char* SQL_PERSONA_ID=(SQL_PERSONA+idString+finalizar).c_str();

    sqlite3_stmt* stmt;
    int rc;

    if(sqlite3_prepare_v2(m_database.getDb(),SQL_PERSONA_ID,-1,&stmt,nullptr)!=SQLITE_OK){
        throw std::runtime_error("Error en la selecion del id");
    }

    rc=sqlite3_step(stmt);
    if(rc==SQLITE_ROW){
        std::string nombrePersona=reinterpret_cast<const char*>(sqlite3_column_text(stmt,0));
        std::string nombrePais=reinterpret_cast<const char*>(sqlite3_column_text(stmt,1));
        PersonaDTO persona(id,nombrePersona,"","","",0,nombrePais);
        persona.getPais();
    }
    if(rc==SQLITE_DONE){
        PersonaDTO persona;
        persona.esNulo();
    }
    sqlite3_finalize(stmt);
}

t_personaPais PersonaDAO::getPersonaExtPorId(int id){
    getPersonaPorId(id);
}


