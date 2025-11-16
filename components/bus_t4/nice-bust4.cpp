#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // для использования вспомогательных функция работ со строками






namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;




CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}


/*
  Listes Commandes OVIEW

  SBS               55 0c 00 ff 00 66 01 05 9D 01 82 01 64 E6 0c
  STOP              55 0c 00 ff 00 66 01 05 9D 01 82 02 64 E5 0c
  OPEN              55 0c 00 ff 00 66 01 05 9D 01 82 03 00 80 0c
  CLOSE             55 0c 00 ff 00 66 01 05 9D 01 82 04 64 E3 0c
  PARENTAL OPEN 1   55 0c 00 ff 00 66 01 05 9D 01 82 05 64 E2 0c
  PARENTAL OPEN 2   55 0c 00 ff 00 66 01 05 9D 01 82 06 64 E1 0c



*/

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;
  if (call.get_stop()) {
    send_cmd(STOP);

  } else if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);

      } else if (newpos == COVER_CLOSED) {
        if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);

      } else { // Position Arbitraire
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Position de conduite requise %d", position_hook_value);
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::setup() {


  _uart =  uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 256, false);
  // Qui est en ligne
//  this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
  

}

void NiceBusT4::loop() {

    if ((millis() - this->last_update_) > 10000) {    // Toutes les 10 sec
// Si le disque n'est pas détecté du premier coup, réessayez plus tard.
        std::vector<uint8_t> unknown = {0x55, 0x55};
        if (this->init_ok == false) {
          this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
          this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //Demande de produit
        }
        
        else if (this->class_gate_ == 0x55) {
		init_device(this->addr_to[0], this->addr_to[1], 0x04);  
	//        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //Demande de produit
	}
        else if (this->manufacturer_ == unknown)  {
                init_device(this->addr_to[0], this->addr_to[1], 0x04);  
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
        //        this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00)); //Demande de produit
		
        }
        this->last_update_ = millis();
    }  // Si chaque minute

	
  // Nous autorisons l'envoi toutes les 100 ms
    uint32_t now = millis();
  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  } 


  while (uart_rx_available(_uart) > 0) {
    uint8_t c = (uint8_t)uart_read_char(_uart);                // lire un octet
    this->handle_char_(c);                                     // Nous envoyons un octet pour traitement.
    this->last_uart_byte_ = now;
  } //Alors que

  if (this->ready_to_tx_) {   // s'il est possible d'envoyer
    if (!this->tx_buffer_.empty()) {  // s'il y a quelque chose à envoyer
      this->send_array_cmd(this->tx_buffer_.front()); // nous envoyons la première commande dans la file d'attente
      this->tx_buffer_.pop();
      this->ready_to_tx_ = false;
    }
  }

  // Sondage sur la position actuelle du lecteur
  if (!is_robus) {
  
  now = millis();
  if (init_ok && (current_operation != COVER_OPERATION_IDLE) && (now - last_position_time > POSITION_UPDATE_INTERVAL)) {
  	last_position_time = now;
    request_position();
  } 
  } // not robus

} //loop


void NiceBusT4::handle_char_(uint8_t c) {
  this->rx_message_.push_back(c);                      // nous ajoutons un octet à la fin du message reçu
  if (!this->validate_message_()) {                    // Nous vérifions le message résultant
    this->rx_message_.clear();                         // Si la vérification échoue, le message est erroné et doit être supprimé.
  }
}


bool NiceBusT4::validate_message_() {                    // vérification du message reçu
  uint32_t at = this->rx_message_.size() - 1;       // numéro du dernier octet reçu
  uint8_t *data = &this->rx_message_[0];               // pointeur vers le premier octet du message
  uint8_t new_byte = data[at];                      // dernier octet reçu

  // Byte 0: HEADER1 (Toujours 0x00)
  if (at == 0)
    return new_byte == 0x00;
  // Byte 1: HEADER2 (Toujours 0x55)
  if (at == 1)
    return new_byte == START_CODE;

  // Byte 2: packet_size - nombre d'octets supplémentaires + 1
  // Aucune vérification n'est effectuée.

  if (at == 2)
    return true;
  uint8_t packet_size = data[2];
  uint8_t length = (packet_size + 3); // la longueur du message attendu est claire


  // Byte 3: Série (ligne) à qui le paquet
  // Aucune vérification n'est effectuée.
  //  uint8_t command = data[3];
  if (at == 3)
    return true;

  // Byte 4: Adresse à laquelle le paquet est envoyé
  // Byte 5: Série (ligne) de qui provient le paquet
  // Byte 6: Adresse de l'expéditeur du paquet
  // Byte 7: Type de message : CMD ou INF
  // Byte 8: Le nombre d'octets moins les deux octets CRC à la fin.

  if (at <= 8)
    // Aucune vérification n'est effectuée.
    return true;

  uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);

  // Byte 9: crc1 = XOR (Byte 3 : Byte 8) XOR des six octets précédents
  if (at == 9)
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Received invalid message checksum 1 %02X!=%02X", data[9], crc1);
      return false;
    }
  // Byte 10:
  // ...

  // Nous attendons que toutes les données du paquet soient reçues.
  if (at  < length)
    return true;

  // nous calculons crc2
  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < length - 1; i++) {
    crc2 = (crc2 ^ data[i]);
  }

  if (data[length - 1] != crc2 ) {
    ESP_LOGW(TAG, "Received invalid message checksum 2 %02X!=%02X", data[length - 1], crc2);
    return false;
  }

  // Byte Last: packet_size
  //  if (at  ==  length) {
  if (data[length] != packet_size ) {
    ESP_LOGW(TAG, "Received invalid message size %02X!=%02X", data[length], packet_size);
    return false;
  }

  // Si vous êtes arrivé jusqu'ici, c'est que le message correct a été reçu et se trouve dans le tampon rx_message_.

  // Supprimez 0x00 du début du message
  rx_message_.erase(rx_message_.begin());

  // pour enregistrer le paquet dans le journal
  std::string pretty_cmd = format_hex_pretty(rx_message_);
  ESP_LOGI(TAG,  "Paquet reçu: %S ", pretty_cmd.c_str() );

  // Nous faisons quelque chose avec ce message.
  parse_status_packet(rx_message_);



  // Renvoyer false pour réinitialiser le tampon de réception
  return false;

}


// nous analysons les paquets reçus
void NiceBusT4::parse_status_packet (const std::vector<uint8_t> &data) {
  if ((data[1] == 0x0d) && (data[13] == 0xFD)) { // erreur
    ESP_LOGE(TAG,  "Cette commande n'est pas disponible pour ce périphérique." );
  }

  if (((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // if evt
  //  ESP_LOGD(TAG, "Paquet de données EVT reçu. Dernière cellule %d ", data[12]);
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string str(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    ESP_LOGI(TAG,  "Ligne de données : %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "données HEX %S ", pretty_data.c_str() );
    // Nous avons reçu un paquet contenant des données EVT, commençons à les analyser.

    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == GET - 0x80) && (data[13] == NOERR)) { // Je souhaite obtenir les réponses complètes aux requêtes GET provenant du lecteur sans erreur.
      ESP_LOGI(TAG,  "Une réponse à votre demande a été reçue %X ", data[10] );
      switch (data[10]) { // cmd_submnu
        case TYPE_M:
          //           ESP_LOGI(TAG,  "Type de lecteur %X",  data[14]);
          switch (data[14]) { //14
            case SLIDING:
              this->class_gate_ = SLIDING;
              //        ESP_LOGD(TAG, "Type de portail : Coulissant %#X ", data[14]);
              break;
            case SECTIONAL:
              this->class_gate_ = SECTIONAL;
              //        ESP_LOGD(TAG, "Type de portail : Sectionnel %#X ", data[14]);
              break;
            case SWING:
              this->class_gate_ = SWING;
              //        ESP_LOGD(TAG, "Type de portail : Battant %#X ", data[14]);
              break;
            case BARRIER:
              this->class_gate_ = BARRIER;
              //        ESP_LOGD(TAG, "Type de porte : Barrière %#X ", data[14]);
              break;
            case UPANDOVER:
              this->class_gate_ = UPANDOVER;
              //        ESP_LOGD(TAG, "Type de portail : Basculant %#X ", data[14]);
              break;
          }  // switch 14
          break; //  TYPE_M
        case INF_IO: // réponse à une demande concernant la position du commutateur de fin de course du portail coulissant
          switch (data[16]) { //16
            case 0x00:
              ESP_LOGI(TAG, "  Le contacteur de fin de course ne fonctionnait pas. ");
              break; // 0x00
            case 0x01:
              ESP_LOGI(TAG, "  Interrupteur de fin de course pour la fermeture ");
              this->position = COVER_CLOSED;
              break; //  0x01
            case 0x02:
              ESP_LOGI(TAG, "  Interrupteur de fin de course pour l'ouverture ");
              this->position = COVER_OPEN;
              break; // 0x02

          }  // switch 16
          this->publish_state_if_changed();  // nous publions le statut

          break; //  INF_IO


        //position d'ouverture maximale de l'encodeur, ouverture, fermeture

        case MAX_OPN:
          if (is_walky) {
            this->_max_opn = data[15];
            this->_pos_opn = data[15];
          }
          else {  
            this->_max_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Position maximale de l'encodeur : %d", this->_max_opn);
          break;

        case POS_MIN:
          this->_pos_cls = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Position de la porte fermée : %d", this->_pos_cls);
          break;

        case POS_MAX:
          if (((data[14] << 8) + data[15])>0x00) { // si la réponse du lecteur contient des données sur la position d'ouverture
          this->_pos_opn = (data[14] << 8) + data[15];}
          ESP_LOGI(TAG, "Position de la porte ouverte : %d", this->_pos_opn);
          break;

        case CUR_POS:
          if (is_walky)
            update_position(data[15]);
          else
            update_position((data[14] << 8) + data[15]);
          break;

        case INF_STATUS:
          switch (data[14]) {
            case OPENED:
              ESP_LOGI(TAG, "  Les portes sont ouvertes");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_OPEN;
              break;
            case CLOSED:
              ESP_LOGI(TAG, "  Les portes sont fermées");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_CLOSED;
              break;
            case 0x01:
              ESP_LOGI(TAG, "  La barrière est arrêtée.");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case 0x00:
              ESP_LOGI(TAG, "  L'état de la porte est inconnu.");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
             case 0x0b:
              ESP_LOGI(TAG, "  Recherche de postes terminés");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
              case STA_OPENING:
              ESP_LOGI(TAG, "  Ouverture en cours");
              this->current_operation = COVER_OPERATION_OPENING;
              break;
              case STA_CLOSING:
              ESP_LOGI(TAG, "  Идёт закрываниеLa fermeture est en cours.");
              this->current_operation = COVER_OPERATION_CLOSING;
              break;
          }  // switch
          this->publish_state_if_changed();  // nous publions le statut
          break;

          //      default: // cmd_mnu
        case AUTOCLS:
          this->autocls_flag = data[14];
	  ESP_LOGCONFIG(TAG, "  Fermeture automatique - L1: %S ", autocls_flag ? "Oui" : "Non");	
          break;
          
        case PH_CLS_ON:
          this->photocls_flag = data[14];
          break;  
          
        case ALW_CLS_ON:
          this->alwayscls_flag = data[14];
          break;  
          
      } // switch cmd_submnu
    } // si les réponses aux requêtes GET reçues du lecteur sont complètes et sans erreur

     if ((data[6] == INF) &&  (data[11] == GET - 0x81) && (data[13] == NOERR)) { // Je m'intéresse aux réponses incomplètes aux requêtes GET reçues sans erreur de toutes les sources.
	ESP_LOGI(TAG,  "Une réponse incomplète à la demande a été reçue. %X, continuation avec décalage %X", data[10], data[12] );
	     // Répétez la commande avec un nouveau décalage.
	tx_buffer_.push(gen_inf_cmd(data[4], data[5], data[9], data[10], GET, data[12]));
     
     } // Réponses incomplètes aux requêtes GET reçues sans erreur du lecteur

	  
    
    if ((data[6] == INF) && (data[9] == FOR_CU)  && (data[11] == SET - 0x80) && (data[13] == NOERR)) { // Je m'intéresse aux réponses aux requêtes SET provenant du lecteur sans erreur.    
      switch (data[10]) { // cmd_submnu
        case AUTOCLS:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, AUTOCLS, GET)); // Fermeture automatique
          break;
          
        case PH_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, PH_CLS_ON, GET)); // Fermer après la photo
          break;  
          
        case ALW_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, ALW_CLS_ON, GET)); // Toujours proche
          break;  
      }// switch cmd_submnu
    }// si les réponses aux requêtes SET provenaient du lecteur sans erreur

    if ((data[6] == INF) && (data[9] == FOR_ALL)  && ((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) { // Je m'intéresse aux réponses FOR_ALL aux requêtes GET arrivées sans erreur.

      switch (data[10]) {
        case MAN:
          //       ESP_LOGCONFIG(TAG, "  Fabricant: %S ", str.c_str());
          this->manufacturer_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case PRD:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // si le paquet provient du récepteur
//            ESP_LOGCONFIG(TAG, "  Récepteur: %S ", str.c_str());
            this->oxi_product.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } // если пакет от приемника
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // si le paquet provient de la logique de commande
//            ESP_LOGCONFIG(TAG, "  Logique: %S ", str.c_str());
            this->product_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
            std::vector<uint8_t> wla1 = {0x57,0x4C,0x41,0x31,0x00,0x06,0x57}; // pour comprendre ce qu'est un Walky Drive
            std::vector<uint8_t> ROBUSHSR10 = {0x52,0x4F,0x42,0x55,0x53,0x48,0x53,0x52,0x31,0x30,0x00}; // pour comprendre que le lecteur ROBUSHSR10
            if (this->product_ == wla1) { 
              this->is_walky = true;
         //     ESP_LOGCONFIG(TAG, "  Moteur WALKY!: %S ", str.c_str());
                                        }
            if (this->product_ == ROBUSHSR10) { 
              this->is_robus = true;
          //    ESP_LOGCONFIG(TAG, "  Moteur ROBUS!: %S ", str.c_str());
                                        }		  
		  
          }
          break;
        case HWR:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // si le paquet provient du récepteur
            this->oxi_hardware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // si le paquet provient de la logique de commande       
          this->hardware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case FRM:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // si le paquet provient du récepteur
            this->oxi_firmware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // si le paquet provient de la logique de commande        
            this->firmware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case DSC:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) { // si le paquet provient du récepteur
            this->oxi_description.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) { // si le paquet provient de la logique de commande          
            this->description_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } //else
          break;
        case WHO:
          if (data[12] == 0x01) {
            if (data[14] == 0x04) { // Moteur
              this->addr_to[0] = data[4];
              this->addr_to[1] = data[5];
              this->init_ok = true;
     //         init_device(data[4], data[5], data[14]);
            }
            else if (data[14] == 0x0A) { // Recepteur
              this->addr_oxi[0] = data[4];
              this->addr_oxi[1] = data[5];
              init_device(data[4], data[5], data[14]);
            }
          }
          break;
      }  // switch

    }  // if  FOR_ALL réponses aux requêtes GET arrivées sans erreur

    if ((data[9] == 0x0A) &&  (data[10] == 0x25) &&  (data[11] == 0x01) &&  (data[12] == 0x0A) &&  (data[13] == NOERR)) { //  paquets provenant du récepteur contenant des informations sur la liste des télécommandes arrivées sans erreur
      ESP_LOGCONFIG(TAG, "numéro de télécommande: %X%X%X%X, equipe: %X, bouton: %X, mode: %X, compteur de frappes: %d", vec_data[5], vec_data[4], vec_data[3], vec_data[2], vec_data[8] / 0x10, vec_data[5] / 0x10, vec_data[7] + 0x01, vec_data[6]);
    }  // if

    if ((data[9] == 0x0A) &&  (data[10] == 0x26) &&  (data[11] == 0x41) &&  (data[12] == 0x08) &&  (data[13] == NOERR)) { //  paquets provenant du récepteur contenant des informations sur le bouton de télécommande lu
      ESP_LOGCONFIG(TAG, "Bouton %X, numéro de télécommande: %X%X%X%X", vec_data[0] / 0x10, vec_data[0] % 0x10, vec_data[1], vec_data[2], vec_data[3]);
    }  // if

  } //  if evt



  //else if ((data[14] == NOERR) && (data[1] > 0x0d)) {  // sinon, le paquet de réponse est une confirmation de la commande reçue
  else if (data[1] > 0x0d) {  // sinon, le paquet de réponse est une confirmation de la commande reçue
    ESP_LOGD(TAG, "Paquet reçu RSP");
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    std::string str(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    ESP_LOGI(TAG,  "Ligne de données: %S ", str.c_str() );
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG,  "Données HEX %S ", pretty_data.c_str() );
    switch (data[9]) { // cmd_mnu
      case FOR_CU:
        ESP_LOGI(TAG, "Ensemble de contrôleur de variateur");
        switch (data[10] + 0x80) { // sub_inf_cmd
          case RUN:
            ESP_LOGI(TAG, "Sous-menu RUN");
			if (data[11] >= 0x80) {
			  switch (data[11] - 0x80) {  // sub_run_cmd1
			    case SBS:
			      ESP_LOGI(TAG, "Commande: SBS");
			      break;
			    case STOP:
			      ESP_LOGI(TAG, "Commande: STOP");
			      break;
			    case OPEN:
			      ESP_LOGI(TAG, "Commande: OPEN");
			      this->current_operation = COVER_OPERATION_OPENING;
			      break;
			    case CLOSE:
			      ESP_LOGI(TAG, "Commande: CLOSE");
			      this->current_operation = COVER_OPERATION_CLOSING;
			      break;
			    case P_OPN1:
			      ESP_LOGI(TAG, "Commande: Ouverture Partielle 1");
			      break;
			    case STOPPED:
			      ESP_LOGI(TAG, "Commande: Arrêté");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    case ENDTIME:
			      ESP_LOGI(TAG, "L'opération a été interrompue en raison d'un délai d'attente dépassé.");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    default:
			      ESP_LOGI(TAG, "Commande Inconnue: %X", data[11]);
			  }  // switch sub_run_cmd1
			} else {
			  switch (data[11]) {  // sub_run_cmd2
			    case STA_OPENING:
			      ESP_LOGI(TAG, "Opération: Ouverture");
			      this->current_operation = COVER_OPERATION_OPENING;
			      break;
			    case STA_CLOSING:
			      ESP_LOGI(TAG, "Opération: Fermeture");
			      this->current_operation = COVER_OPERATION_CLOSING;
			      break;
			    case CLOSED:
			      ESP_LOGI(TAG, "Opération: Fermée");
			      this->current_operation = COVER_OPERATION_IDLE;
			      this->position = COVER_CLOSED;
			      break;
			    case OPENED:
			      ESP_LOGI(TAG, "Opération: Ouvert");
			      this->current_operation = COVER_OPERATION_IDLE;
			      this->position = COVER_OPEN;
			      // calibrate opened position if the motor does not report max supported position (Road 400)
                  if (this->_max_opn == 0) {
                    this->_max_opn = this->_pos_opn = this->_pos_usl;
                    ESP_LOGI(TAG, "Opened position calibrated");
                  }
			      break;
			    case STOPPED:
			      ESP_LOGI(TAG, "Opération: Arrêtée");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    case PART_OPENED:
			      ESP_LOGI(TAG, "Opération: Partiellement ouvert");
			      this->current_operation = COVER_OPERATION_IDLE;
			      request_position();
			      break;
			    default:
			      ESP_LOGI(TAG, "Opération inconnue: %X", data[11]);
			  }  // switch sub_run_cmd2
			}
			this->publish_state_if_changed();  // nous publions le statut
            break; //RUN

          case STA:
            ESP_LOGI(TAG,  "Sous-menu État en mouvement" );
            switch (data[11]) { // sub_run_cmd2
              case STA_OPENING:
              case 0x83: // Road 400
                ESP_LOGI(TAG, "Mouvement: S'ouvre" );
                this->current_operation = COVER_OPERATION_OPENING;
                break;
              case STA_CLOSING:
              case 0x84: // Road 400
                ESP_LOGI(TAG,  "Mouvement: Se ferme" );
                this->current_operation = COVER_OPERATION_CLOSING;
                break;
              case CLOSED:
                ESP_LOGI(TAG,  "Mouvement: Fermé" );
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_CLOSED;
                break;
              case OPENED:
                ESP_LOGI(TAG, "Mouvement: Ouvert");
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_OPEN;
                break;
              case STOPPED:
                ESP_LOGI(TAG, "Mouvement: Arrêté");
                this->current_operation = COVER_OPERATION_IDLE;
                request_position();
                break;
              default: // sub_run_cmd2
                ESP_LOGI(TAG,  "Mouvement: %X", data[11] );

                
            } // switch sub_run_cmd2

            update_position((data[12] << 8) + data[13]);
            break; //STA

          default: // sub_inf_cmd
            ESP_LOGI(TAG,  "Sous-menu %X", data[10] );
        }  // switch sub_inf_cmd

        break; // Ensemble de contrôleur de variateur
      case CONTROL:
        ESP_LOGI(TAG,  "Pacquet CONTROL" );
        break; // CONTROL
      case FOR_ALL:
        ESP_LOGI(TAG,  "Un forfait pour tous" );
        break; // FOR_ALL
      case 0x0A:
        ESP_LOGI(TAG,  "Récepteur" );
        break; // Paquet récepteur
      default: // cmd_mnu
        ESP_LOGI(TAG,  "Menu %X", data[9] );
    }  // switch  cmd_mnu


  } // else

 if ((data[6] == CMD) && (data[9] == FOR_CU)  && (data[10] == CUR_MAN) && (data[13] == NOERR)) { // Nous nous intéressons aux réponses FOR_CU aux requêtes CMD reçues sans erreur. Nous recherchons le statut de RO600.

  ///////////////////////////////////////////////////////////////////////////////////


  // RSP est une réponse (ReSPonce) à la réception d'une simple commande CMD, et non à son exécution. Elle signale également la fin de l'opération.
  /* if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) && (data[12] == 0x19)) { // Nous reconnaissons le paquet d'état par son contenu en octets spécifiques.
     //  ESP_LOGD(TAG, "Paquet reçu RSP. cmd = %#x", data[11]);
*/
     switch (data[11]) {
       case STA_OPENING:
         this->current_operation = COVER_OPERATION_OPENING;
         ESP_LOGD(TAG, "Statut: Ouverture");
         break;
       case STA_CLOSING:
         this->current_operation = COVER_OPERATION_CLOSING;
         ESP_LOGD(TAG, "Statut: Fermeture");
         break;
       case OPENED:
         this->position = COVER_OPEN;
         ESP_LOGD(TAG, "Statut: Ouvert");
         this->current_operation = COVER_OPERATION_IDLE;
         break;
       case CLOSED:
         this->position = COVER_CLOSED;
         ESP_LOGD(TAG, "Statut: Fermé");
         this->current_operation = COVER_OPERATION_IDLE;
         break;
       case STOPPED:
         this->current_operation = COVER_OPERATION_IDLE;
         ESP_LOGD(TAG, "Statut: Arrêté");
         break;

     }  // switch

     this->publish_state();  // nous publions le statut

    } //if
 
  /*
    // statut après avoir atteint les limites
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) &&  (data[12] == 0x00)) { // Nous reconnaissons le paquet d'état par son contenu en octets spécifiques.
      ESP_LOGD(TAG, "Paquet de fin de course reçu. État = %#x", data[11]);
      switch (data[11]) {
        case OPENED:
          this->position = COVER_OPEN;
          ESP_LOGD(TAG, "Statut: Ouvert");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          ESP_LOGD(TAG, "Statut: Fermé");
          this->current_operation = COVER_OPERATION_IDLE;
          break;
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Statut: Ouverture");
          break;
        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Statut: Fermeture");
          break;
      } //switch
      this->publish_state();  // nous publions le statut
    } //if
  */
  // STA = 0x40,   // statut en mouvement
  /*
    if ((data[1] == 0x0E) && (data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == STA) ) { // Nous reconnaissons le paquet d'état par son contenu en octets spécifiques.
      uint16_t ipos = (data[12] << 8) + data[13];
      ESP_LOGD(TAG, "Manœuvre actuelle: %#X Position: %#X %#X, ipos = %#x,", data[11], data[12], data[13], ipos);
      this->position = ipos / 2100.0f; // nous transmettons la position au composant

      switch (data[11]) {
        case OPENING:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Statut: Ouverture");
          break;

        case OPENING2:
          this->current_operation = COVER_OPERATION_OPENING;
          ESP_LOGD(TAG, "Statut: Ouverture");
          break;

        case CLOSING:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Statut: Fermeture");
          break;
        case CLOSING2:
          this->current_operation = COVER_OPERATION_CLOSING;
          ESP_LOGD(TAG, "Statut: Fermeture");
          break;
        case OPENED:
          this->position = COVER_OPEN;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Statut: Ouvert");
          //      this->current_operation = COVER_OPERATION_OPENING;
          //    ESP_LOGD(TAG, "Statut: Ouverture");
          break;
        case CLOSED:
          this->position = COVER_CLOSED;
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Statut: Fermé");
          //      this->current_operation = COVER_OPERATION_CLOSING;
          //ESP_LOGD(TAG, "Statut: Fermeture");
          break;
        case STOPPED:
          this->current_operation = COVER_OPERATION_IDLE;
          ESP_LOGD(TAG, "Statut : Arrêté");
          break;

      }  // switch

      this->publish_state();  // nous publions le statut

    } //if
  */


  ////////////////////////////////////////////////////////////////////////////////////////
} // function







void NiceBusT4::dump_config() {    //  Ajoutez des informations sur le contrôleur connecté au journal.
  ESP_LOGCONFIG(TAG, "  Bus T4 Cover");
  /*ESP_LOGCONFIG(TAG, "  Address: 0x%02X%02X", *this->header_[1], *this->header_[2]);*/
  switch (this->class_gate_) {
    case SLIDING:
      ESP_LOGCONFIG(TAG, "  Type : Portails coulissants");
      break;
    case SECTIONAL:
      ESP_LOGCONFIG(TAG, "  Type : Portes sectionnelles");
      break;
    case SWING:
      ESP_LOGCONFIG(TAG, "  Type : Portails battants");
      break;
    case BARRIER:
      ESP_LOGCONFIG(TAG, "  Type : Barrière");
      break;
    case UPANDOVER:
      ESP_LOGCONFIG(TAG, "  Type : Portails basculants");
      break;
    default:
      ESP_LOGCONFIG(TAG, "  Type : Porte inconnue, 0x%02X", this->class_gate_);
  } // switch


  ESP_LOGCONFIG(TAG, "  Position maximale de l'encodeur ou du minuteur: %d", this->_max_opn);
  ESP_LOGCONFIG(TAG, "  Position de la porte ouverte: %d", this->_pos_opn);
  ESP_LOGCONFIG(TAG, "  Position de la porte fermée: %d", this->_pos_cls);

  std::string manuf_str(this->manufacturer_.begin(), this->manufacturer_.end());
  ESP_LOGCONFIG(TAG, "  Fabricant: %S ", manuf_str.c_str());

  std::string prod_str(this->product_.begin(), this->product_.end());
  ESP_LOGCONFIG(TAG, "  Moteur: %S ", prod_str.c_str());

  std::string hard_str(this->hardware_.begin(), this->hardware_.end());
  ESP_LOGCONFIG(TAG, "  Modèle: %S ", hard_str.c_str());

  std::string firm_str(this->firmware_.begin(), this->firmware_.end());
  ESP_LOGCONFIG(TAG, "  Micrologiciel: %S ", firm_str.c_str());
  
  std::string dsc_str(this->description_.begin(), this->description_.end());
  ESP_LOGCONFIG(TAG, "  Description du moteur: %S ", dsc_str.c_str());


  ESP_LOGCONFIG(TAG, "  Adresse de la passerelle: 0x%02X%02X", addr_from[0], addr_from[1]);
  ESP_LOGCONFIG(TAG, "  Adresse du lecteur: 0x%02X%02X", addr_to[0], addr_to[1]);
  ESP_LOGCONFIG(TAG, "  Adresse du destinataire: 0x%02X%02X", addr_oxi[0], addr_oxi[1]);
  
  std::string oxi_prod_str(this->oxi_product.begin(), this->oxi_product.end());
  ESP_LOGCONFIG(TAG, "  Récepteur: %S ", oxi_prod_str.c_str());
  
  std::string oxi_hard_str(this->oxi_hardware.begin(), this->oxi_hardware.end());
  ESP_LOGCONFIG(TAG, "  Matériel récepteur: %S ", oxi_hard_str.c_str());

  std::string oxi_firm_str(this->oxi_firmware.begin(), this->oxi_firmware.end());
  ESP_LOGCONFIG(TAG, "  micrologiciel du récepteur: %S ", oxi_firm_str.c_str());
  
  std::string oxi_dsc_str(this->oxi_description.begin(), this->oxi_description.end());
  ESP_LOGCONFIG(TAG, "  Description du récepteur: %S ", oxi_dsc_str.c_str());
 
  ESP_LOGCONFIG(TAG, "  Fermeture automatique - L1: %S ", autocls_flag ? "Oui" : "Non");
  ESP_LOGCONFIG(TAG, "  Photo rapprochée - L2: %S ", photocls_flag ? "Oui" : "Non");
  ESP_LOGCONFIG(TAG, "  Toujours proche - L3: %S ", alwayscls_flag ? "Oui" : "Non");
  
}




//formation du paquet envoyé au moteur
std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {this->addr_to[0], this->addr_to[1], this->addr_from[0], this->addr_from[1]}; // En tête
  frame.push_back(CMD);  // 0x01
  frame.push_back(0x05);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(CONTROL);
  frame.push_back(RUN);
  frame.push_back(control_cmd);
  frame.push_back(0x64); // OFFSET CMD, DPRO924 refusait de fonctionner avec 0x00, bien que les autres lecteurs aient répondu aux commandes.
  uint8_t crc2 = (frame[7] ^ frame[8] ^ frame[9] ^ frame[10]);
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // pour consigner la commande dans le journal
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "L'équipe est formée : %S ", pretty_cmd.c_str() );

  return frame;
}

// Création d'une commande INF avec et sans données
std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, const uint8_t whose, const uint8_t inf_cmd, const uint8_t run_cmd, const uint8_t next_data, const std::vector<uint8_t> &data, size_t len) {
  std::vector<uint8_t> frame = {to_addr1, to_addr2, this->addr_from[0], this->addr_from[1]}; // En tête
  frame.push_back(INF);  // 0x08 mes_type
  frame.push_back(0x06 + len); // mes_size
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(whose);
  frame.push_back(inf_cmd);
  frame.push_back(run_cmd);
  frame.push_back(next_data); // next_data
  frame.push_back(len);
  if (len > 0) {
    frame.insert(frame.end(), data.begin(), data.end()); // bloc de données
  }
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < 12 + len; i++) {
    crc2 = crc2 ^ frame[i];
  }
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);

  // pour consigner la commande dans le journal
  //  std::string pretty_cmd = format_hex_pretty(frame);
  //  ESP_LOGI(TAG,  "paquet INF généré: %S ", pretty_cmd.c_str() );

  return frame;

}


void NiceBusT4::send_raw_cmd(std::string data) {

  std::vector < uint8_t > v_cmd = raw_cmd_prepare (data);
  send_array_cmd (&v_cmd[0], v_cmd.size());

}


//  Il faut ajouter ici une vérification des données utilisateur incorrectes.
std::vector<uint8_t> NiceBusT4::raw_cmd_prepare (std::string data) { // préparation des données saisies par l'utilisateur en vue de leur soumission
// Supprimez tout sauf les lettres et les chiffres hexadécimaux.
data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch) {
    return (!(isxdigit(ch)) );
  }), data.end()); 

  //assert (data.size () % 2 == 0); // vérifier la parité
  std::vector < uint8_t > frame;
  frame.resize(0); // Réinitialiser la taille de l'équipe

  for (uint8_t i = 0; i < data.size (); i += 2 ) { // remplir le tableau de commandes
    std::string sub_str(data, i, 2); // nous prenons 2 octets de la commande
    char hexstoi = (char)std::strtol(&sub_str[0], 0 , 16); // convertir en nombre
    frame.push_back(hexstoi);  // nous écrivons un nombre dans l'élément ligne de la nouvelle commande
  }


  return frame;

}



void NiceBusT4::send_array_cmd (std::vector<uint8_t> data) {          // envoie break + la commande préalablement préparée dans le tableau
  return send_array_cmd((const uint8_t *)data.data(), data.size());
}
void NiceBusT4::send_array_cmd (const uint8_t *data, size_t len) {
  // envoi de données à l'UART

  char br_ch = 0x00;                                               // pour la pause
  uart_flush(_uart);                                               // Effacement de l'UART
  uart_set_baudrate(_uart, BAUD_BREAK);                            // Soyons courageux.
  uart_write(_uart, &br_ch, 1);                                    // Nous envoyons zéro à basse vitesse, zéro long
  //uart_write(_uart, (char *)&dummy, 1);
  uart_wait_tx_empty(_uart);                                       // Nous attendons la fin de la transmission. Une erreur s'est produite dans la bibliothèque uart.h (noyau esp8266 3.0.2) ; l'attente ne suffit pas pour poursuivre l'appel à uart_set_baudrate().
  delayMicroseconds(90);                                           // Ajoutez un délai d'attente, sinon la vitesse changera avant l'envoi. Avec un délai sur le d1-mini, j'ai obtenu un signal parfait (rupture = 520 µs).
  uart_set_baudrate(_uart, BAUD_WORK);                             // Nous remettons le Bodrait en état de marche.
  uart_write(_uart, (char *)&data[0], len);                        // Nous envoyons le paquet principal.
  //uart_write(_uart, (char *)raw_cmd_buf, sizeof(raw_cmd_buf));
  uart_wait_tx_empty(_uart);                                       // Nous attendons que l'expédition soit terminée.



  std::string pretty_cmd = format_hex_pretty((uint8_t*)&data[0], len);                    // pour consigner la commande dans le journal
  ESP_LOGI(TAG,  "Envoyé: %S ", pretty_cmd.c_str() );

}


// Générer et envoyer des commandes INF à partir d'une configuration YAML
void NiceBusT4::send_inf_cmd(std::string to_addr, std::string whose, std::string command, std::string type_command, std::string next_data, bool data_on, std::string data_command) {
  std::vector < uint8_t > v_to_addr = raw_cmd_prepare (to_addr);
  std::vector < uint8_t > v_whose = raw_cmd_prepare (whose);
  std::vector < uint8_t > v_command = NiceBusT4::raw_cmd_prepare (command);
  std::vector < uint8_t > v_type_command = raw_cmd_prepare (type_command);
  std::vector < uint8_t > v_next_data = raw_cmd_prepare (next_data);
  std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);


  if (data_on) {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0], v_data_command, v_data_command.size()));
  } else {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], v_type_command[0], v_next_data[0]));
  } // else
}

// Générer et envoyer des commandes d'installation au contrôleur de disque à partir d'une configuration YAML avec un minimum de paramètres
void NiceBusT4::set_mcu(std::string command, std::string data_command) {
    std::vector < uint8_t > v_command = raw_cmd_prepare (command);
    std::vector < uint8_t > v_data_command = raw_cmd_prepare (data_command);
    tx_buffer_.push(gen_inf_cmd(0x04, v_command[0], 0xa9, 0x00, v_data_command));
  }
  
// initialisation du périphérique
void NiceBusT4::init_device (const uint8_t addr1, const uint8_t addr2, const uint8_t device ) {
  if (device == FOR_CU) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00)); // type de demande de lecteur
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00)); // demande du fabricant
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  demande de firmware
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //demande de produit
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //demande de matériel
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));   //demande de poste vacant
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00)); // demande de poste de clôture
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //demande de description
    if (is_walky)  // Demander la valeur maximale pour l'encodeur
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00, {0x01}, 1));
    else
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));
    request_position();  // demande de poste actuel
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00)); //État de la porte (Ouverte/Fermée/Arrêtée)
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, AUTOCLS, GET, 0x00)); // Fermeture automatique
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, PH_CLS_ON, GET, 0x00)); // Fermer après la photo
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, ALW_CLS_ON, GET, 0x00)); // Toujours proche
  }
  if (device == FOR_OXI) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00)); //demande de produit
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00)); //demande de matériel 
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00)); //  demande de firmware
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00)); //demande de description
  }
  
}

// Demande de position d'entraînement de courant conditionnel
void NiceBusT4::request_position(void) {
  if (is_walky)
    tx_buffer_.push(gen_inf_cmd(this->addr_to[0], this->addr_to[1], FOR_CU, CUR_POS, GET, 0x00, {0x01}, 1));
  else
    tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));
}

// Mise à jour de la position de conduite actuelle
void NiceBusT4::update_position(uint16_t newpos) {
  last_position_time = millis();
  _pos_usl = newpos;
  position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
  ESP_LOGI(TAG, "Position conditionnelle de la porte: %d, position dans %%: %.3f", newpos, position);
  if (position < CLOSED_POSITION_THRESHOLD) position = COVER_CLOSED;
  publish_state_if_changed();  // nous publions le statut
  
  if ((position_hook_type == STOP_UP && _pos_usl >= position_hook_value) || (position_hook_type == STOP_DOWN && _pos_usl <= position_hook_value)) {
  	ESP_LOGI(TAG, "La position requise a été atteinte. Nous fermons la barrière.");
  	send_cmd(STOP);
  	position_hook_type = IGNORE;
  }
}

// Publication de l'état de la porte lors de son changement
void NiceBusT4::publish_state_if_changed(void) {
  if (current_operation == COVER_OPERATION_IDLE) position_hook_type = IGNORE;
  if (last_published_op != current_operation || last_published_pos != position) {
    publish_state();
    last_published_op = current_operation;
    last_published_pos = position;
  }
}

}  // namespace bus_t4
}  // namespace esphome
