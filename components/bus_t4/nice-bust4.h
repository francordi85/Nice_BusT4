/*
  Nice BusT4
  Échange de données UART à 19200 8n1
  Avant le paquet de données, une pause d'une durée de 519 µs (10 bits) est envoyée.
  Le contenu du paquet compris est décrit dans la structure packet_cmd_body_t.

 

  Pour Oview, 80 est toujours ajouté à l'adresse.
  L'adresse du contrôleur de portail reste inchangée.

Подключение

BusT4                       ESP8266

Paroi de l'appareil        Rx Tx GND
9  7  5  3  1  
10 8  6  4  2
Conecteur
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V




Extrait du manuel nice_dmbm_integration_protocol.pdf

• ADR : Il s’agit de l’adresse réseau NICE où se trouvent les périphériques que vous souhaitez gérer. Elle peut prendre une valeur comprise entre 1 et 63 (1 à 3F)..
Cette valeur doit être en hexadécimal. Si la destination est un module d'intégration DIN-BAR, cette valeur est 0 (adr = 0), si la destination est
est un moteur intelligent, cette valeur est 1 (adr = 1).
• EPT : Il s’agit de l’adresse du moteur Nice inclus dans l’ADR du réseau. Sa valeur peut être comprise entre 1 et 127 et doit être au format hexadécimal.
• CMD : Il s'agit de la commande que vous souhaitez envoyer à la destination (ADR, EPT).
• PRF : Commande de configuration du profil.
• FNC : Il s’agit de la fonction que vous souhaitez envoyer à la destination (ADR, EPT).
• EVT : Il s’agit d’un événement envoyé à la destination (ADR, EPT).



*/


#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"           // ajouter une action
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h"              // analyse de chaînes de caractères avec des outils intégrés
#include <queue>                               // travailler avec la file d'attente



namespace esphome {
namespace bus_t4 {

/* pour s'adresser brièvement aux membres de la classe */
using namespace esphome::cover;
//using esp8266::timeoutTemplate::oneShotMs;


static const int _UART_NO=UART0; /* номер uart */
static const int TX_P = 1;         /* пин Tx */
static const uint32_t BAUD_BREAK = 9200; /* débit binaire pendant une longue impulsion avant l'éclatement */
static const uint32_t BAUD_WORK = 19200; /* Bodrait de travail */
static const uint8_t START_CODE = 0x55; /*octet de début de paquet */

static const float CLOSED_POSITION_THRESHOLD = 0.007;  // Valeur de la position du mécanisme en pourcentage, en dessous de laquelle le portail est considéré comme complètement fermé.
static const uint32_t POSITION_UPDATE_INTERVAL = 500;  // Intervalle de mise à jour de la position actuelle du disque, ms

/* Paramètres réseau ESP
  La série peut prendre des valeurs de 0 à 63, la valeur par défaut étant 0.
  L'adresse OVIEW commence par 8

  Lors de la mise en réseau de plusieurs disques OXI, des lignes différentes doivent être spécifiées pour chaque disque.
  Dans ce cas, la rangée OXI doit être identique au variateur qu'elle contrôle.
*/






/* Type de message par paquet
  Pour l'instant, nous nous intéressons uniquement à CMD et INF.
  Je n'ai pas étudié le reste en détail et je n'ai pas vérifié les chiffres.
  6e octet des paquets CMD et INF
*/
enum mes_type : uint8_t {
  CMD = 0x01,  /* Le numéro a été vérifié, des commandes ont été envoyées au système d'automatisation. */
//  LSC = 0x02,  /* travailler avec des listes de scripts */
//  LST = 0x03,  /* travailler automatiquement avec les listes */
//  POS = 0x04,  /* demande et modification de la position d'automatisation */
//  GRP = 0x05,  /* envoi de commandes au groupe d'automatisation avec le masque de bits du moteur spécifié */
//  SCN = 0x06,  /* travailler avec des scripts */
//  GRC = 0x07,  /* Envoi de commandes à un groupe d'automatisations créées via l'outil de configuration Nice Screen */
  INF = 0x08,  /* renvoie ou définit des informations sur l'appareil */
//  LGR = 0x09,  /* travailler avec des listes de groupes */
//  CGR = 0x0A,  /* travailler avec des catégories de groupes créées via l'outil de configuration Nice Screen */
};




/* 
menu de commandes dans la hiérarchie oview
9e octet des paquets CMD
*/
enum cmd_mnu  : uint8_t {
  CONTROL = 0x01,
};


/* utilisé dans les réponses STA*/
enum sub_run_cmd2 : uint8_t {
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
       OPENED = 0x04,
       CLOSED = 0x05,
      ENDTIME = 0x06,  // La manœuvre a été interrompue en raison d'un délai d'attente.
      STOPPED = 0x08,
  PART_OPENED = 0x10,  // ouverture partielle
};

/* Erreurs */
enum errors_byte  : uint8_t {
  NOERR = 0x00, // Aucune erreur
  FD = 0xFD,    // Il n'existe aucune commande pour ce périphérique.
  };

// Types de moteurs
enum motor_type  : uint8_t {
  SLIDING = 0x01, 
  SECTIONAL = 0x02,
  SWING = 0x03,
  BARRIER = 0x04,
  UPANDOVER = 0x05, // up-and-over portails basculants
  };

//  neuvième octet
enum whose_pkt  : uint8_t {
  FOR_ALL = 0x00,  /* paquet pour/de la part de tous */
  FOR_CU = 0x04,  /* paquet pour/depuis l'unité de commande */
  FOR_OXI = 0x0A,  /* paquet pour/depuis un récepteur OXI */
  };
	
// le dixième octet des paquets GET/SET EVT, pour les paquets CMD uniquement, la valeur a été rencontrée RUN
enum command_pkt  : uint8_t {
  TYPE_M         = 0x00,   /* Demande de type de lecteur */
  INF_STATUS     = 0x01, //	État de la porte (Ouverte/Fermée/Arrêtée)	
  WHO	         = 0x04,  /* Qui est en ligne ?     */
  MAC            = 0x07,    // mac address.
  MAN            = 0x08,   // Fabricant
  PRD            = 0x09,   // Produit.
  INF_SUPPORT    = 0x10, //  Commandes INF disponibles
  HWR            = 0x0a,   // hardware version.
  FRM            = 0x0b,   // firmware version.
  DSC            = 0x0c,   // description.
  CUR_POS        = 0x11,  // La position conditionnelle actuelle de l'automatisation, DPRO924, attend ensuite que les positions soient définies.
  MAX_OPN        = 0x12,   // Ouverture maximale possible par encodeur.
  POS_MAX        = 0x18,   // Position maximale (ouverture) selon l'encodeur
  POS_MIN        = 0x19,   // Position minimale (fermeture) selon l'encodeur
  INF_P_OPN1     = 0x21, //	Ouverture partielle1 
  INF_P_OPN2     = 0x22, //	Ouverture partielle2
  INF_P_OPN3     = 0x23, //	Ouverture partielle3
  INF_SLOW_OPN   = 0x24, // Ouverture lente
  INF_SLOW_CLS   = 0x25, // Fermeture lente	
  OPN_OFFSET     = 0x28, /* Retard d'ouverture  open offset */
  CLS_OFFSET     = 0x29, /* Retard à la fermeture  close offset */
  OPN_DIS        = 0x2a, /* Paramètres principaux - Ouverture et déchargement Open discharge */
  CLS_DIS        = 0x2b, /* Paramètres principaux - Fermeture du déchargement Close discharge */
  REV_TIME       = 0x31, /* Paramètres principaux - Durée inverse (Brief inversion value) */
  OPN_PWR        = 0x4A,    /* Paramètres principaux - Contrôle de la force - Force d'ouverture */	  	  	  	  	  
  CLS_PWR        = 0x4B,    /* Paramètres principaux - Contrôle de la force - Force de fermeture */	  	  	  	  	  	  
  SPEED_OPN      = 0x42,    /* Paramètres principaux - Réglages de vitesse - Vitesse d'ouverture */	  	  	  	  	  	  	  
  SPEED_CLS      = 0x43,    /* Paramètres principaux - Réglage de la vitesse - Vitesse de fermeture */	  
  SPEED_SLW_OPN  = 0x45,    /* Paramètres de base - Paramètres de vitesse - Vitesse d'ouverture lente */	
  SPEED_SLW_CLS  = 0x46,    /* Paramètres principaux - Réglage de la vitesse - Vitesse de fermeture lente */	
  OUT1           = 0x51,  /* Configuration des sorties 1 */	  
  OUT2           = 0x52,  /* Configuration des sorties 2 */	  	  
  LOCK_TIME      = 0x5A,  /* Paramètres de sortie - Verrouiller le temps de fonctionnement */
  S_CUP_TIME     = 0x5C,  /* Paramètres de sortie - Durée de la ventouse */	  
  LAMP_TIME      = 0x5B,  /* Paramètres de sortie - Durée de fonctionnement de la lampe d'éclairage de courtoisie light Time*/
  COMM_SBS       = 0x61,  /* Configuration des commandes - Étape par étape */	  
  COMM_POPN      = 0x62,  /* Configuration des commandes - Ouverture partielle */	  	  
  COMM_OPN       = 0x63,  /* Configuration des commandes - Ouvrir */	  	  	  
  COMM_CLS       = 0x64,  /* Configuration des commandes - Fermer */	  
  COMM_STP       = 0x65,  /* Configuration des commandes - STOP */		  
  COMM_PHOTO     = 0x68,  /* Configuration des commandes - Photos */		  
  COMM_PHOTO2    = 0x69,  /* Configuration des commandes - Photo 2 */
  COMM_PHOTO3    = 0x6A,  /* Configuration des commandes - Photo 3 */
  COMM_OPN_STP   = 0x6B,  /* Configuration des commandes - Arrêt à l'ouvertureи */	  
  COMM_CLS_STP   = 0x6C,  /* Configuration des commandes - Arrêt à la fermeture */	 
  IN1            = 0x71,  /* Configuration des entrées 1 */
  IN2            = 0x72,  /* Configuration des entrées 2 */
  IN3            = 0x73,  /* Configuration des entrées 3 */
  IN4            = 0x74,  /* Configuration des entrées 4 */
  COMM_LET_OPN   = 0x78,  /* Configuration des commandes - Interférence avec l'ouverture */	  	  	  
  COMM_LET_CLS   = 0x79,  /* Configuration des commandes - Fermeture des interférences */	  	  	  	  

  AUTOCLS        = 0x80,    /* Paramètres de base - Fermeture automatique */
  P_TIME         = 0x81,    /* Paramètres de base - Temps de pause */
  PH_CLS_ON      = 0x84,    /* Paramètres de base - Fermer après la prise de vue - Actif */	  
  PH_CLS_VAR     = 0x86,    /* Paramètres de base - Fermer après la photo - Mode */	  	  
  PH_CLS_TIME    = 0x85,    /* Paramètres de base - Fermer après la prise de vue - Délai d'expiration */	  	  	  
  ALW_CLS_ON     = 0x88,    /* Paramètres de base - Toujours fermer - Actif */	  	  
  ALW_CLS_VAR    = 0x8A,    /* Paramètres de base - Toujours fermer - Mode */	  
  ALW_CLS_TIME   = 0x89,    /* Paramètres de base - Toujours fermer - Délai d'expiration */	  	  	  
  STAND_BY_ACT   = 0x8c,    /* Paramètres de base - Mode veille - Actif  ON / OFF */
  WAIT_TIME      = 0x8d,    /* Paramètres de base - Mode veille - Durée de veille */
  STAND_BY_MODE  = 0x8e,    /* Paramètres de base - Mode veille - Mode -  safety = 0x00, bluebus=0x01, all=0x02*/
  START_ON       = 0x90,    /* Paramètres de base - Paramètres de démarrage - Actif */		  	  
  START_TIME     = 0x91,    /* Paramètres de base - Paramètres de démarrage - Heure de début */		  	  	  
  SLOW_ON        = 0xA2,    /* Paramètres de base - Ralentissement */	
  DIS_VAL        = 0xA4,    /* Position - La valeur est invalide, désactiver la valeur */

  BLINK_ON       = 0x94,    /* Paramètres de base - Clignotement - Actif */		  	  	  	  
  BLINK_OPN_TIME = 0x95,    /* Paramètres principaux - Pré-scintillement - Temps d'ouverture */		  	  	  	  	  
  BLINK_CLS_TIME = 0x99,    /* Paramètres principaux - Pré-scintillement - Heure de fermeture */
  OP_BLOCK       = 0x9a,    /* Paramètres principaux - Blocage du moteur (Operator block)*/
  KEY_LOCK       = 0x9c,    /* Paramètres de base - Verrouillage des boutons */
  T_VAL          = 0xB1,    /* Valeur seuil d'alarme Seuil avant intervention en nombre de manœuvres */
  P_COUNT        = 0xB2,    /* décompte partiel */
  C_MAIN         = 0xB4,    /* Annulation de la maintenance */
  DIAG_BB        = 0xD0,    /* DIAGNOSTICS of bluebus devices */  
  INF_IO         = 0xD1,    /* état d'entrée-sortie	*/
  DIAG_PAR       = 0xD2,    /* DIAGNOSTICS of other parameters   */
  
  
  
  


  

  CUR_MAN = 0x02,  // Manœuvre actuelle
  SUBMNU  = 0x04,  // Sous-menu
  STA = 0xC0,   // statut en mouvement
  MAIN_SET = 0x80,   // Paramètres principaux
  RUN = 0x82,   // Commande à exécuter

  };	

	
/* run cmd 11-й octets du paquet EVT */
enum run_cmd  : uint8_t {
  SET = 0xA9,  /* demande de modification des paramètres */
  GET = 0x99,   /* demande de paramètres */
  GET_SUPP_CMD = 0x89, /* obtenir les commandes prises en charge */
  };


/* La commande à exécuter.   
11e octet du paquet CMD
Utilisé dans les requêtes et les réponses */
enum control_cmd : uint8_t { 
  SBS = 0x01,    /* Step by Step */
  STOP = 0x02,   /* Stop */
  OPEN = 0x03,   /* Open */
  CLOSE = 0x04,  /* Close */
  P_OPN1 = 0x05, /* Partial opening 1 - ouverture partielle, mode guichet */
  P_OPN2 = 0x06, /* Partial opening 2 */
  P_OPN3 = 0x07, /* Partial opening 3 */
  RSP = 0x19, /* Réponse de l'interface accusant réception de la commande */
  EVT = 0x29, /* Réponse de l'interface envoyant les informations demandées */
 
  P_OPN4 = 0x0b, /* Partial opening 4 - Collectivement */
  P_OPN5 = 0x0c, /* Partial opening 5 - Priorité SBS */
  P_OPN6 = 0x0d, /* Partial opening 6 - Ouvrir et verrouiller */
  UNLK_OPN = 0x19, /* Déverrouiller et ouvrir */
  CLS_LOCK = 0x0E, /* Fermer et verrouiller */
  UNLCK_CLS = 0x1A, /*  Déverrouiller et fermer */
  LOCK = 0x0F, /* Verrouiller */
  UNLOCK = 0x10, /* Deverouiller */
  LIGHT_TIMER = 0x11, /* Minuteur d'éclairage */
  LIGHT_SW = 0x12, /* Éclairage allumé/éteint */
  HOST_SBS = 0x13, /* SBS Maître */
  HOST_OPN = 0x14, /* Ouverture Maître*/
  HOST_CLS = 0x15, /* Fermeture Maître */
  SLAVE_SBS = 0x16, /* SBS Esclave */
  SLAVE_OPN = 0x17, /* Ouverture Esclave */
  SLAVE_CLS = 0x18, /* Fermeture Esclave */
  AUTO_ON = 0x1B, /* L'ouverture automatique est activée */
  AUTO_OFF = 0x1C, /* L'ouverture automatique n'est pas active */
  
};
	
	
	
	
	
	
/* Informations permettant de mieux comprendre la composition des paquets dans le protocole */
// Corps du paquet de requête CMD
// paquets dont la taille du corps est de 0x0c = 12 octets 
	/*
struct packet_cmd_body_t {
  uint8_t byte_55;              // En-tête, toujours 0x55
  uint8_t pct_size1;            // Taille du corps du paquet (en-tête et CRC exclus. Nombre total d'octets moins trois), pour les commandes = 0x0c
  uint8_t for_series;           // série pour qui le paquet ff = tout le monde
  uint8_t for_address;          // Adresse à qui le colis ff est envoyé = tout le monde
  uint8_t from_series;          // série de qui est le paquet
  uint8_t from_address;         // adresse de l'expéditeur du colis
  uint8_t mes_type;             // Type de message : 1 = CMD, 8 = INF
  uint8_t mes_size;             // le nombre d'octets supplémentaires moins les deux octets CRC à la fin, pour les commandes = 5
  uint8_t crc1;                 // CRC1, XOR des six octets précédents
  uint8_t cmd_mnu;              // Menu de commandes. cmd_mnu = 1 pour les commandes de contrôle
  uint8_t setup_submnu;         // Le sous-menu, associé au groupe de commandes, détermine le type de message envoyé.
  uint8_t control_cmd;          // La commande à exécuter
  uint8_t offset;               // Décalage de réponse. Affecte les requêtes telles que la liste des commandes prises en charge.
  uint8_t crc2;            		// crc2, XOR des quatre octets précédents
  uint8_t pct_size2;            // Taille du corps du paquet (en-tête et CRC exclus. Nombre total d'octets moins trois), pour les commandes = 0x0c

};





// Corps du paquet de réponse RSP
// paquets dont la taille du corps est de 0x0e=14 octets
struct packet_rsp_body_t {
  uint8_t byte_55;              // En-tête, toujours 0x55
  uint8_t pct_size1;            // Taille du corps du paquet (en-tête et CRC exclus. Nombre total d'octets moins trois), >= 0x0e
  uint8_t to_series;            // série pour qui le paquet ff = tout le monde
  uint8_t to_address;           // Adresse à qui le colis ff est envoyé = tout le monde
  uint8_t from_series;          // série de qui est le paquet
  uint8_t from_address;         // adresse de l'expéditeur du paquet
  uint8_t mes_type;             // type de message, pour ces paquets toujours 8 = INF
  uint8_t mes_size;             // le nombre d'octets supplémentaires moins les deux octets CRC à la fin, pour les commandes = 5
  uint8_t crc1;                 // CRC1, XOR des six octets précédents
  uint8_t cmd_mnu;              // Menu de commandes. cmd_mnu = 1 pour les commandes de contrôle
  uint8_t sub_inf_cmd;          // Le sous-menu d'origine de la commande indique que sa valeur est inférieure de 0x80 à celle du sous-menu initial.
  uint8_t sub_run_cmd;          // Quelle commande a été reçue ? La valeur est supérieure de 0x80 à la commande reçue.
  uint8_t hb_data;              // données, bit le plus significatif
  uint8_t lb_data;              // données, bit de poids faible
  uint8_t err;                  // Erreurs
  uint8_t crc2;                 // crc2, XOR des quatre octets précédents
  uint8_t pct_size2;            // Taille du corps du paquet (en-tête et CRC exclus, nombre total d'octets moins trois), >= 0x0e

};
	
 // Corps du paquet de réponse de données EVT
 
 struct packet_evt_body_t {
  uint8_t byte_55;              // En-tête, toujours 0x55
  uint8_t pct_size1;            // Taille du corps du paquet (en-tête et CRC exclus, nombre total d'octets moins trois), >= 0x0e
  uint8_t to_series;            // série pour qui le paquet ff = tout le monde
  uint8_t to_address;           // Adresse à qui le colis ff est envoyé = tout le monde
  uint8_t from_series;          // série de qui est le paquet
  uint8_t from_address;         // adresse de l'expéditeur du colis
  uint8_t mes_type;             // type de message, pour ces paquets toujours 8 = INF
  uint8_t mes_size;             // le nombre d'octets supplémentaires moins les deux octets CRC à la fin, pour les commandes = 5
  uint8_t crc1;                 // CRC1, XOR des six octets précédents
  uint8_t whose;                // Pack de quel produit ? Options : 00 - général, 04 - contrôleur de variateur, 0A - récepteur OXI
  uint8_t setup_submnu;         // Le sous-menu d'origine de la commande est indiqué. La valeur correspond au sous-menu d'origine.
  uint8_t sub_run_cmd;          // À quelle commande répondons-nous ? La valeur est inférieure de 0x80 à la commande précédemment envoyée.
  uint8_t next_data;            // Bloc de données suivant
  uint8_t err;                  // Erreurs
  uint8_t data_blk;             // Bloc de données, peut occuper plusieurs octets
  uint8_t crc2;                 // crc2, XOR de tous les octets précédents jusqu'au neuvième (Paquet de qui)
  uint8_t pct_size2;            // Taille du corps du paquet (en-tête et CRC exclus, nombre total d'octets moins trois), >= 0x0e

};
 
 
*/

enum position_hook_type : uint8_t {
     IGNORE = 0x00,
    STOP_UP = 0x01,
  STOP_DOWN = 0x02
 };

// Je crée une classe qui hérite des membres des classes Component et Cover.
class NiceBusT4 : public Component, public Cover {
  public:
	
    // paramètres du lecteur
    bool autocls_flag; // Fermeture automatique - L1
    bool photocls_flag; // Photo rapprochée - L2
    bool alwayscls_flag; // Toujours proche - L3
    bool init_ok = false; // Détection du lecteur à la mise sous tension
    bool is_walky = false; // La commande de demande de position est différente pour le mode walky.
    bool is_robus = false; // Robus ne nécessite pas de demandes de positionnement périodiques.
    bool is_ro = false; // Pour le ro600, le contenu du package, incluant l'état de position et l'état de mouvement, diffère.
		
    void setup() override;
    void loop() override;
    void dump_config() override; // pour consigner les informations relatives à l'équipement dans le journal.

    void send_raw_cmd(std::string data);
    void send_cmd(uint8_t data) {this->tx_buffer_.push(gen_control_cmd(data));}	
    void send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command,  std::string next_data, bool data_on, std::string data_command); // commande longue
    void set_mcu(std::string command, std::string data_command); // commande au contrôleur de moteur
		

    void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }
    
 /*   void set_update_interval(uint32_t update_interval) {  // intervalle de récupération de l'état du lecteur
      this->update_interval_ = update_interval;
    }*/

    cover::CoverTraits get_traits() override;

  protected:
    void control(const cover::CoverCall &call) override;
    void send_command_(const uint8_t *data, uint8_t len);
    void request_position(void);  // Demande de position d'entraînement de courant conditionnel
    void update_position(uint16_t newpos);  // Mise à jour de la position de conduite actuelle

    uint32_t last_position_time{0};  // Date de la dernière mise à jour de la position actuelle
    uint32_t update_interval_{500};
    uint32_t last_update_{0};
    uint32_t last_uart_byte_{0};

    CoverOperation last_published_op;  // Dernières informations publiées concernant le statut et la position
    float last_published_pos{-1};

    void publish_state_if_changed(void);

    uint8_t position_hook_type{IGNORE};  // Drapeau et position du réglage de position du variateur
    uint16_t position_hook_value;

    uint8_t class_gate_ = 0x55; // 0x01 sliding, 0x02 sectional, 0x03 swing, 0x04 barrier, 0x05 up-and-over
//    uint8_t last_init_command_;
	
    bool init_cu_flag = false;	
    bool init_oxi_flag = false;	

	
    // variables pour l'UART
    uint8_t _uart_nr;
    uart_t* _uart = nullptr;
    uint16_t _max_opn = 0;     // position maximale de l'encodeur ou de la minuterie
    uint16_t _pos_opn = 2048;  // Position d'ouverture de l'encodeur ou du minuteur, non compatible avec tous les variateurs.
    uint16_t _pos_cls = 0;     // Position de fermeture de l'encodeur ou du minuteur, non compatible avec tous les variateurs
    uint16_t _pos_usl = 0;     // position actuelle conditionnelle de l'encodeur ou du temporisateur, non disponible pour tous les variateurs
    // paramètres de l'en-tête du paquet généré
    uint8_t addr_from[2] = {0x00, 0x66}; // De qui provient le paquet, adresse de la passerelle Bust4
    uint8_t addr_to[2]; // = 0x00ff;	 // à qui le paquet est envoyé, l'adresse du contrôleur de variateur qui est contrôlé
    uint8_t addr_oxi[2]; // = 0x000a;	 // adresse du destinataire

    std::vector<uint8_t> raw_cmd_prepare (std::string data);             // préparation des données saisies par l'utilisateur en vue de leur soumission
	
    // génération de commandes INF
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len);	 // tous les champs
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd) {return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, 0x00, {0x00}, 0 );} // pour les commandes sans données
    std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, std::vector<uint8_t> data){
	    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, run_cmd, next_data, data, data.size());} // pour les commandes avec des données
    std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data){
	    return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, next_data, {0x00}, 0);} // pour les commandes avec adresse et sans données
    	    
    // génération de commandes cmd
    std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);	    	
	
    void init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device );
    void send_array_cmd (std::vector<uint8_t> data);	
    void send_array_cmd (const uint8_t *data, size_t len);


    void parse_status_packet (const std::vector<uint8_t> &data); // Analysons le package d'état
    
    void handle_char_(uint8_t c);                                       // gestionnaire d'octets reçus
    void handle_datapoint_(const uint8_t *buffer, size_t len);          // processeur de données reçues
    bool validate_message_();                                           // fonction de vérification des messages reçus

    std::vector<uint8_t> rx_message_;                          			// Ici, le message reçu est accumulé octet par octet.
    std::queue<std::vector<uint8_t>> tx_buffer_;             			// file d'attente des commandes à envoyer
    bool ready_to_tx_{true};	                           				// drapeau pour la possibilité d'envoyer des commandes
	
    std::vector<uint8_t> manufacturer_ = {0x55, 0x55};  // Fabricant inconnu lors de l'initialisation
    std::vector<uint8_t> product_;
    std::vector<uint8_t> hardware_;
    std::vector<uint8_t> firmware_;
    std::vector<uint8_t> description_;	
    std::vector<uint8_t> oxi_product;
    std::vector<uint8_t> oxi_hardware;
    std::vector<uint8_t> oxi_firmware;
    std::vector<uint8_t> oxi_description;	

}; //класс

} // namespace bus_t4
} // namespace esphome
