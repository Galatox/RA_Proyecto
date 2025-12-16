//    Módulo: dto.hpp
// Propósito: Declaración de las clases DTO (Data Transfer Object)
//            que sirven para comunicar las clases del "negocio"
//            (las que modelan entidades o conceptos del problema)
//            con las clases DAO (Data Access Object)
//     Autor: Javier Macías
//     Fecha: 14/03/2025

#ifndef DTO_HPP
#define DTO_HPP

#include <string>

class PaisDTO
{
  public:
    PaisDTO();  //Pais "nulo"
    PaisDTO(std::string codigo, std::string nombre,
            int telf, std::string contacto);
    std::string getCodigo();
    std::string getNombre();
    int getTelf();
    std::string getContacto();
    bool esNulo();

  private:
    std::string m_codigo;  //cód. ISO
    std::string m_nombre;
    int m_telf;
    std::string m_contacto;  //nombre persona contacto
    bool m_esNulo;
};

class PersonaDTO
{
  public:
    PersonaDTO();  //Persona "nula"
    PersonaDTO(int id, std::string nombre, std::string sexo,
               std::string dniNie, std::string fnac,
               int altura, std::string pais);
    int getId();
    std::string getNombre();
    std::string getSexo();
    std::string getDniNie();
    std::string getFnac();
    int getAltura();
    std::string getPais();
    bool esNulo();

  private:
    int m_id;
    std::string m_nombre;
    std::string m_sexo;
    std::string m_dniNie;
    std::string m_fnac;
    int m_altura; //en cm
    std::string m_pais;
    bool m_esNulo;
};

#endif  //DTO_HPP
