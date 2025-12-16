#include "dto.hpp"
#include <iostream>

/// Clase PaisDTO
PaisDTO::PaisDTO():m_esNulo(true){
}

PaisDTO::PaisDTO(std::string codigo, std::string nombre,int telf,std::string contacto):m_codigo(codigo),m_nombre(nombre),m_telf(telf),m_contacto(contacto),m_esNulo(false){
}

//GETs PaisDTO
std::string PaisDTO::getCodigo(){
	return m_codigo;
}
std::string PaisDTO::getNombre(){
	return m_nombre;
}
int PaisDTO::getTelf(){
	return m_telf;
}
std::string PaisDTO::getContacto(){
	return m_contacto;
}
bool PaisDTO::esNulo(){
	return m_esNulo;
}

///  Clase PersonaDTO

PersonaDTO::PersonaDTO():m_esNulo(true){
}

PersonaDTO::PersonaDTO(int id, std::string nombre, std::string sexo, std::string dniNie, std::string fnac,int altura,std::string pais):m_id(id),m_nombre(nombre),m_sexo(sexo),m_dniNie(dniNie),m_altura(altura),m_pais(pais),m_esNulo(false){
}

//GETs PersonaDTO
int PersonaDTO::getId(){
	return m_id;
}
std::string PersonaDTO::getNombre(){
	return m_nombre;	
}
std::string PersonaDTO::getSexo(){
	return m_sexo;
}
std::string PersonaDTO::getDniNie(){
	return m_dniNie;	
}
std::string PersonaDTO::getFnac(){
	return m_fnac;	
}
int PersonaDTO::getAltura(){
	return m_altura;	
}
std::string PersonaDTO::getPais(){
	return m_pais;	
}
bool PersonaDTO::esNulo(){
	return m_esNulo;
}
	