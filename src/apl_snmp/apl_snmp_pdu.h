/*
 * apl_snmp_pdu.h
 *
 *  Created on: 8 thg 11, 2023
 *      Author: mobifone
 */

#ifndef APL_SNMP_PDU_H_
#define APL_SNMP_PDU_H_

//define=========================================================
#define SNMP_OCTETSTRING_MAX_LENGTH              1024
#define MAX_OID_LENGTH                           128
#define MIN_RANDOM                               0x00
#define MAX_RANDOM                               0xFE
#define NO_ERR                                   0
#define ERR                                      1
//

//
typedef enum ASN_TYPE_WITH_VALUE
{
    // Primitives
    INTEGER             = 0x02,
    STRING              = 0x04,
    NULLTYPE            = 0x05,
    OID                 = 0x06,

    // Complex
    STRUCTURE           = 0x30,
    NETWORK_ADDRESS     = 0x40,
    COUNTER32           = 0x41,
    GAUGE32             = 0x42, // UNSIGNED32
    TIMESTAMP           = 0x43,
    OPAQUE              = 0x44,
    COUNTER64           = 0x46,

    NOSUCHOBJECT        = 0x80,
    NOSUCHINSTANCE      = 0x81,
    ENDOFMIBVIEW        = 0x82,

    GetRequestPDU       = 0xA0,
    GetNextRequestPDU   = 0xA1,
    GetResponsePDU      = 0xA2,
    SetRequestPDU       = 0xA3,
    TrapPDU             = 0xA4,
    GetBulkRequestPDU   = 0xA5,
    Trapv2PDU           = 0xA7
} ASN_TYPE;

typedef struct{
    uint8_t             mess_header;
    uint8_t             length_of_pdu;

    uint8_t             ver_data_type;
    uint8_t             length_of_version;
    uint8_t             snmp_version;         //0-snmpv1; 1-snmp v2

    uint8_t             community_data_type;
    uint8_t             length_of_community;
    uint8_t             community_data[20];   //public default for snmp v2

    uint8_t             pdu_type;
    uint8_t             length_of_pdu_type;

    uint8_t             id_req_data_type;
    uint8_t             length_of_id_req;
    uint8_t             id_req_data[4];

    uint8_t             err_status_data_type;
    uint8_t             length_of_err_status;
    uint8_t             err_status_data;

    uint8_t             err_index_data_type;
    uint8_t             length_of_err_index;
    uint8_t             err_status_index;

    uint8_t             start_var_sequence;
    uint8_t             length_of_start_var_sequence;
    uint8_t             start_first_var;

    uint8_t             length_of_first_var;
    uint8_t             oid_data_type;
    uint8_t             leng_of_oid;
    uint8_t             oid_value[30];

    //uint8_t             end_of_oid_value;
    uint8_t             end_of_var;

    uint8_t             end_of_mess_header;

    uint8_t             value_set[50];
    uint8_t             type_value_set;
    uint8_t             value_set_length;
}PDU_SNMP_REQ_T;

typedef struct{
    PDU_SNMP_REQ_T pdu_req;
    uint8_t        data_req[256];
    uint8_t        index_data_req;
}SNMP_MANAGER_T;
extern  SNMP_MANAGER_T snmp_manager_t;

typedef struct{
	uint8_t var_[100];
	uint8_t var_len;
	uint8_t type_var_;

}SNMP_VAR_T;

extern  SNMP_VAR_T snmp_var_out;
extern  SNMP_VAR_T snmp_var_set;
//function=========================================================
uint8_t snmp_get_var(SNMP_MANAGER_T *p_snmp, char *oid_string, char *community_str,int32_t *int_var_out, char *string_var_out, uint8_t *string_var_out_len , uint8_t type_var,  uint16_t delay);
uint8_t snmp_set_var(SNMP_MANAGER_T *p_snmp,char *oid_string, char *community_str_set,int32_t  int_var_in, char *string_var_in ,int32_t  *int_var_out, char *string_var_out,uint8_t *string_var_out_len, uint8_t type_var, uint16_t delay);
uint8_t snmp_get_var_to_reg(SNMP_MANAGER_T *p_snmp, char *oid_string, char *community_str,int32_t *int_var_out, char *string_var_out, uint8_t *string_var_out_len , uint8_t type_var,  uint16_t delay, uint16_t* reg,uint8_t *cnt_filter);
uint8_t handler_snmp(void);

#endif /* APL_SNMP_PDU_H_ */
