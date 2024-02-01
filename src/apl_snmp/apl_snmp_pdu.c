/*
 * apl_snmp_pdu.c
 *
 *  Created on: Apr 26, 2023
 *      Author: thanh ca ca
 */


#include"apl_header_files.h"
///====================================================================================================================================
extern MD_STATUS R_SCI1_AsyncTransmit (uint8_t * const tx_buf,uint16_t tx_num);
///====================================================================================================================================
extern uint8_t rx1_buff[300];
extern volatile uint16_t  g_sci1_rx_count;
///====================================================================================================================================
static uint8_t  snmp_req(SNMP_MANAGER_T *p_snmp,char *oid_string, char *community_str );
static uint8_t snmp_rx(uint8_t *buf, uint16_t length_buff, char *oid_string, char *community_str, uint8_t* var_out,uint8_t *var_out_len,uint8_t *type_value_out);
static uint8_t snmp_gen_setting(SNMP_MANAGER_T *p_snmp,char *oid_string, char *community_str, uint8_t *value, uint8_t value_len, uint8_t type_value );
static uint8_t snmp_convert_var_2_snmp_var(int32_t  int_var, char *string_var, SNMP_VAR_T *p_snmp_var,uint8_t type_var  );
static uint8_t snmp_convert_snmp_var_2_var(SNMP_VAR_T *p_snmp_var, int32_t  *int_var, char *string_var,uint8_t *string_var_len,  uint8_t type_var);


///1.==================================================================================================================================
static int random(int minN, int maxN) {
	return minN + rand() % (maxN + 1 - minN);
}


static uint8_t random_ID() {
	uint8_t ret = 0;
	ret = (uint8_t) (random(MIN_RANDOM, MAX_RANDOM));
	return ret;
}


static uint8_t snmp_convert_oid(char *oid_string, uint8_t *out_oid) {

	uint32_t buff_oid[30];
	memset(buff_oid, 0, sizeof(buff_oid));
	uint8_t buff_oid_index = 0;
	uint8_t oid_length = 0;

	char *tok = strtok(oid_string, ".");

	while (tok != NULL) {
		buff_oid[buff_oid_index++] = (uint32_t) atoi(tok);
		tok = strtok(NULL, ".");
	}
	*(out_oid + 0) = (uint8_t) (buff_oid[0] * 40 + buff_oid[1]);
	oid_length++;
	for (uint8_t i = 2; i < buff_oid_index; i++) {
		if (buff_oid[i] <= 0x7F) {
			*(out_oid + oid_length) = (uint8_t) buff_oid[i];
			oid_length++;
		} else {
			if (buff_oid[i] < 0x4000) {
				*(out_oid + oid_length) = (uint8_t) (((buff_oid[i] >> 7) & (0x007F)) + 0x80);
				oid_length++;
				*(out_oid + oid_length) = (uint8_t) (buff_oid[i] & (0x007F));
				oid_length++;
			} else if (buff_oid[i] >= 0x4000) {
				*(out_oid + oid_length) = (uint8_t) (((buff_oid[i] >> 14) & (0x007F)) + 0x80);
				oid_length++;
				*(out_oid + oid_length) = (uint8_t) (((buff_oid[i] >> 7) & (0x007F)) + 0x80);
				oid_length++;
				*(out_oid + oid_length) = (uint8_t) (buff_oid[i] & (0x007F));
				oid_length++;
			}
		}
	}
	return oid_length;
}


///2.==========================================================================================================================
static uint8_t  snmp_req(SNMP_MANAGER_T *p_snmp,char *oid_string, char *community_str ){
    memset(&p_snmp->pdu_req,0,sizeof(p_snmp->pdu_req));
    memset(p_snmp->data_req,0,sizeof(p_snmp->data_req));

    p_snmp->index_data_req = 0;

    p_snmp->pdu_req.mess_header                 = 0x30;
    p_snmp->pdu_req.length_of_pdu               = 0x00;

    p_snmp->pdu_req.ver_data_type               = INTEGER;
    p_snmp->pdu_req.length_of_version           = 0x01;
    p_snmp->pdu_req.snmp_version                = 0x01;                                //0-snmpv1; 1- snmp v2
	p_snmp->pdu_req.length_of_pdu 				= p_snmp->pdu_req.length_of_pdu + 3;

    p_snmp->pdu_req.community_data_type         = STRING;
    p_snmp->pdu_req.length_of_community         = (uint8_t)strlen((const char *)community_str);
    sprintf((char *)p_snmp->pdu_req.community_data,"%s","");
    sprintf((char *)p_snmp->pdu_req.community_data,"%s",(const char *)community_str); //public default for snmp v2
    p_snmp->pdu_req.length_of_pdu               = (uint8_t)(p_snmp->pdu_req.length_of_pdu + 2 + p_snmp->pdu_req.length_of_community);

    p_snmp->pdu_req.pdu_type                    = GetRequestPDU;
    p_snmp->pdu_req.length_of_pdu_type          = 0x00;
    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu +2;

    p_snmp->pdu_req.id_req_data_type            = INTEGER;
    p_snmp->pdu_req.length_of_id_req            = 0x04;
	for (uint8_t i = 0; i < 4; i++) {
		p_snmp->pdu_req.id_req_data[i] = random_ID();
	}
    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu + 6;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type + 6;

    p_snmp->pdu_req.err_status_data_type        = INTEGER;
    p_snmp->pdu_req.length_of_err_status        = 0x01;
    p_snmp->pdu_req.err_status_data             = 0x00;

    p_snmp->pdu_req.err_index_data_type         = INTEGER;
    p_snmp->pdu_req.length_of_err_index         = 0x01;
    p_snmp->pdu_req.err_status_index            = 0x00;

    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu + 6;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type + 6;

    p_snmp->pdu_req.start_var_sequence          = 0x30;
    p_snmp->pdu_req.length_of_start_var_sequence= 0x00;

    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu + 2;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type + 2;

    p_snmp->pdu_req.start_first_var             = 0x30;
    p_snmp->pdu_req.length_of_first_var         = 0x00;

    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu + 2;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type +2;
    p_snmp->pdu_req.length_of_start_var_sequence= p_snmp->pdu_req.length_of_start_var_sequence + 2;

    p_snmp->pdu_req.oid_data_type               = OID;
    p_snmp->pdu_req.leng_of_oid                 = snmp_convert_oid(oid_string,p_snmp->pdu_req.oid_value);

	p_snmp->pdu_req.length_of_pdu 					= (uint8_t) (p_snmp->pdu_req.length_of_pdu + p_snmp->pdu_req.leng_of_oid + 2);
	p_snmp->pdu_req.length_of_pdu_type 				= (uint8_t) (p_snmp->pdu_req.length_of_pdu_type + p_snmp->pdu_req.leng_of_oid + 2);
	p_snmp->pdu_req.length_of_start_var_sequence 	= (uint8_t) (p_snmp->pdu_req.length_of_start_var_sequence + p_snmp->pdu_req.leng_of_oid + 2);
	p_snmp->pdu_req.length_of_first_var 			= (uint8_t) (p_snmp->pdu_req.length_of_first_var + p_snmp->pdu_req.leng_of_oid + 2);

    //p_snmp->pdu_req.oid_value[30];
    p_snmp->pdu_req.end_of_var                  = NULLTYPE;
    p_snmp->pdu_req.end_of_mess_header          = 0x00;

    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu + 2;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type + 2;
    p_snmp->pdu_req.length_of_start_var_sequence= p_snmp->pdu_req.length_of_start_var_sequence + 2;
    p_snmp->pdu_req.length_of_first_var         = p_snmp->pdu_req.length_of_first_var + 2;

    //============================================================================================
    p_snmp->index_data_req = 0;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.mess_header;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_pdu;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.ver_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_version;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.snmp_version;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.community_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_community;
	for (uint8_t i = 0; i < p_snmp->pdu_req.length_of_community; i++) {
		p_snmp->data_req[p_snmp->index_data_req++] = p_snmp->pdu_req.community_data[i];
	}
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.pdu_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_pdu_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.id_req_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_id_req;
	for (uint8_t i = 0; i < p_snmp->pdu_req.length_of_id_req; i++) {
		p_snmp->data_req[p_snmp->index_data_req++] = p_snmp->pdu_req.id_req_data[i];
	}
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.err_status_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_err_status;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.err_status_data;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.err_index_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_err_index;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.err_status_index;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.start_var_sequence;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_start_var_sequence;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.start_first_var;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_first_var;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.oid_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.leng_of_oid;
	for (uint8_t i = 0; i < p_snmp->pdu_req.leng_of_oid; i++) {
		p_snmp->data_req[p_snmp->index_data_req++] = p_snmp->pdu_req.oid_value[i];
	}
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.end_of_var;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.end_of_mess_header;

	if (p_snmp->index_data_req < p_snmp->pdu_req.length_of_pdu)
		return ERR;

    return NO_ERR;
}


///3.============================================================================================================================
static uint8_t snmp_rx(uint8_t *buf, uint16_t length_buff, char *oid_string, char *community_str, uint8_t* var_out,uint8_t *var_out_len,uint8_t *type_value_out){
	uint16_t idx;
	uint8_t oid[30];
	idx = 0;
	if(buf[idx]!= 0x30) return ERR;
	idx = 1;
	if(buf[idx]!= (length_buff -2)) return ERR;
	idx = idx +1;
	if (buf[idx]!= INTEGER) return ERR;
	idx = idx +1;
	idx = idx + buf[idx]+1;

	if (buf[idx]!= STRING) return ERR;
	idx = idx +1;
	if(buf[idx]!=strlen(community_str)) return ERR;

	for(uint8_t i=0; i<strlen(community_str);i++){
		if(buf[idx+i+1]!= community_str[i]) return ERR;
	}

	idx = idx + buf[idx]+1;

	if (buf[idx]!= GetResponsePDU) return ERR;
	idx = idx +1;
	idx = idx +1;
	if (buf[idx]!= INTEGER ) return ERR;
	idx = idx +1;
	idx = idx + buf[idx]+1;
	if (buf[idx]!=INTEGER) return ERR;
	idx = idx +1;
	if(buf[idx]!=0x01) return ERR;
	idx = idx +1;
	if(buf[idx]!=0x00) return ERR;
	idx = idx +1;

	if (buf[idx]!=INTEGER) return ERR;
	idx = idx +1;
	if(buf[idx]!=0x01) return ERR;
	idx = idx +1;
	if(buf[idx]!=0x00) return ERR;
	idx = idx +1;

	if(buf[idx]!=0x30) return ERR;
	idx = idx +2;
	if(buf[idx]!=0x30) return ERR;
	idx = idx +2;
	if(buf[idx]!=OID) return ERR;
	idx = idx+1;

	if(buf[idx]!=snmp_convert_oid(oid_string,oid)) return ERR;

	for(uint8_t i =0;i<buf[idx];i++)
	{
		if(buf[idx+i+1]!= oid[i]) return ERR;
	}

	idx = idx + buf[idx]+1;

	*type_value_out = buf[idx];
	idx=idx+1;
	*var_out_len    = buf[idx];
	for(uint8_t i=0; i<buf[idx];i++)
	{
		var_out[i] = buf[idx+1+i];
	}
	idx = idx + buf[idx];

	if (idx!=(length_buff-1)) return ERR;

    return NO_ERR;

}

///4.===========================================================================================================================
static uint8_t snmp_gen_setting(SNMP_MANAGER_T *p_snmp,char *oid_string, char *community_str, uint8_t *value, uint8_t value_len, uint8_t type_value ){

	memset(&p_snmp->pdu_req,0,sizeof(p_snmp->pdu_req));
	memset(p_snmp->data_req,0,sizeof(p_snmp->data_req));

	p_snmp->index_data_req = 0;

	p_snmp->pdu_req.mess_header                 = 0x30;
	p_snmp->pdu_req.length_of_pdu               = 0x00;

	p_snmp->pdu_req.ver_data_type               = INTEGER;
	p_snmp->pdu_req.length_of_version           = 0x01;
	p_snmp->pdu_req.snmp_version                = 0x01;                                //0-snmpv1; 1- snmp v2
	p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu +3;

	p_snmp->pdu_req.community_data_type         = STRING;
	p_snmp->pdu_req.length_of_community         = (uint8_t)strlen((const char *)community_str);
	sprintf((char *)p_snmp->pdu_req.community_data,"%s","");
	sprintf((char *)p_snmp->pdu_req.community_data,"%s",(const char *)community_str); //public default for snmp v2
	p_snmp->pdu_req.length_of_pdu               = (uint8_t)(p_snmp->pdu_req.length_of_pdu +2 + p_snmp->pdu_req.length_of_community);

	p_snmp->pdu_req.pdu_type                    =  SetRequestPDU;
	p_snmp->pdu_req.length_of_pdu_type          = 0x00;
    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu +2;

    p_snmp->pdu_req.id_req_data_type            = INTEGER ;
    p_snmp->pdu_req.length_of_id_req            = 0x04;
    for(uint8_t i= 0; i<4;i++)                  { p_snmp->pdu_req.id_req_data[i] = random_ID();}
    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu +6;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type +6;

    p_snmp->pdu_req.err_status_data_type        = INTEGER;
    p_snmp->pdu_req.length_of_err_status        = 0x01;
    p_snmp->pdu_req.err_status_data             = 0x00;;
    p_snmp->pdu_req.err_index_data_type         = INTEGER;
    p_snmp->pdu_req.length_of_err_index         = 0x01;
    p_snmp->pdu_req.err_status_index            = 0x00;
    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu +6;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type +6;

    p_snmp->pdu_req.start_var_sequence          = 0x30;
    p_snmp->pdu_req.length_of_start_var_sequence= 0x00;
    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu +2;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type +2;

    p_snmp->pdu_req.start_first_var             = 0x30;
    p_snmp->pdu_req.length_of_first_var         = 0x00;
    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu +2;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type +2;
    p_snmp->pdu_req.length_of_start_var_sequence= p_snmp->pdu_req.length_of_start_var_sequence+2;

    p_snmp->pdu_req.oid_data_type               = OID;
    p_snmp->pdu_req.leng_of_oid                 = snmp_convert_oid(oid_string,p_snmp->pdu_req.oid_value);

    p_snmp->pdu_req.length_of_pdu               = (uint8_t)(p_snmp->pdu_req.length_of_pdu + p_snmp->pdu_req.leng_of_oid+2);
    p_snmp->pdu_req.length_of_pdu_type          = (uint8_t)(p_snmp->pdu_req.length_of_pdu_type + p_snmp->pdu_req.leng_of_oid+2);
    p_snmp->pdu_req.length_of_start_var_sequence= (uint8_t)(p_snmp->pdu_req.length_of_start_var_sequence + p_snmp->pdu_req.leng_of_oid+2);
    p_snmp->pdu_req.length_of_first_var         = (uint8_t)(p_snmp->pdu_req.length_of_first_var + p_snmp->pdu_req.leng_of_oid+2);

    p_snmp->pdu_req.type_value_set              = type_value;
    p_snmp->pdu_req.value_set_length            = value_len;

    for(uint8_t i =0; i< value_len; i++ ){
    	p_snmp->pdu_req.value_set[i] = value[i];
    }

    p_snmp->pdu_req.length_of_pdu               = p_snmp->pdu_req.length_of_pdu + 2
    		+ value_len;
    p_snmp->pdu_req.length_of_pdu_type          = p_snmp->pdu_req.length_of_pdu_type + 2+value_len;
    p_snmp->pdu_req.length_of_start_var_sequence= p_snmp->pdu_req.length_of_start_var_sequence + 2+value_len;
    p_snmp->pdu_req.length_of_first_var         = p_snmp->pdu_req.length_of_first_var + 2+ value_len;

    //============================================================================================
    p_snmp->index_data_req = 0;
    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.mess_header;
    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.length_of_pdu;

    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.ver_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.length_of_version;
    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.snmp_version;

    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.community_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.length_of_community;

    for(uint8_t i=0; i<p_snmp->pdu_req.length_of_community;i++){
        p_snmp->data_req[p_snmp->index_data_req++]     =p_snmp->pdu_req.community_data[i];
    }


    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.pdu_type;
    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.length_of_pdu_type;
    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.id_req_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         =p_snmp->pdu_req.length_of_id_req;

    for(uint8_t i=0; i<p_snmp->pdu_req.length_of_id_req;i++) {
        p_snmp->data_req[p_snmp->index_data_req++]     = p_snmp->pdu_req.id_req_data[i];
    }
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.err_status_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_err_status;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.err_status_data;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.err_index_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_err_index;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.err_status_index;

    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.start_var_sequence;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_start_var_sequence;

    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.start_first_var;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.length_of_first_var;

    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.oid_data_type;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.leng_of_oid;

    for(uint8_t i=0;i<p_snmp->pdu_req.leng_of_oid;i++){
        p_snmp->data_req[p_snmp->index_data_req++]     = p_snmp->pdu_req.oid_value[i];
    }

    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.type_value_set;
    p_snmp->data_req[p_snmp->index_data_req++]         = p_snmp->pdu_req.value_set_length;

    for(uint8_t i =0; i<p_snmp->pdu_req.value_set_length; i++)
    {
    	p_snmp->data_req[p_snmp->index_data_req++]     = p_snmp->pdu_req.value_set[i];
    }

    if(p_snmp->index_data_req<p_snmp->pdu_req.length_of_pdu) return ERR;

	return NO_ERR;

}


///5.======================================================================================================================
uint8_t snmp_get_var(SNMP_MANAGER_T *p_snmp, char *oid_string, char *community_str,int32_t *int_var_out, char *string_var_out, uint8_t *string_var_out_len , uint8_t type_var,  uint16_t delay){
	uint8_t err_snmp = NO_ERR;
	SNMP_VAR_T p_snmp_var_out;

	err_snmp = snmp_req(p_snmp, oid_string, community_str);
	if (err_snmp!= NO_ERR) return err_snmp;

	RS485_M_Ctr = 1U; //RS485 send mode
	R_SCI1_AsyncTransmit(p_snmp->data_req,p_snmp->index_data_req);
	RS485_M_Ctr = 0U; //RS485 receive mode
	//len =  sizeof(rx_buff);
	memset(rx1_buff, 0, sizeof(rx1_buff));
	R_Config_SCI1_Serial_Receive((uint8_t*) &rx1_buff, sizeof(rx1_buff) - 1);

	R_BSP_SoftwareDelay(delay, BSP_DELAY_MILLISECS); //100;

	memset(&p_snmp_var_out, 0, sizeof(&p_snmp_var_out));
	err_snmp = snmp_rx(rx1_buff,g_sci1_rx_count, oid_string,community_str, p_snmp_var_out.var_, &p_snmp_var_out.var_len, &p_snmp_var_out.type_var_);
    if (err_snmp == ERR) return  err_snmp;
	err_snmp = snmp_convert_snmp_var_2_var(&p_snmp_var_out,int_var_out,string_var_out,string_var_out_len,type_var);

	return err_snmp;
}


///6.=========================================================================================================================
uint8_t snmp_set_var(SNMP_MANAGER_T *p_snmp,char *oid_string, char *community_str_set,int32_t  int_var_in, char *string_var_in ,int32_t  *int_var_out, char *string_var_out,uint8_t *string_var_out_len, uint8_t type_var, uint16_t delay){

	uint8_t err_snmp = NO_ERR;

	SNMP_VAR_T p_snmp_var_set,p_snmp_var_out;

	err_snmp = snmp_convert_var_2_snmp_var(int_var_in,string_var_in,&p_snmp_var_set,type_var);
	if(err_snmp != NO_ERR ) return err_snmp;

	err_snmp = snmp_gen_setting(p_snmp, oid_string, community_str_set, p_snmp_var_set.var_,p_snmp_var_set.var_len, p_snmp_var_set.type_var_);
    if(err_snmp != NO_ERR ) return err_snmp;

	RS485_M_Ctr = 1U; //RS485 send mode
	R_SCI1_AsyncTransmit(p_snmp->data_req, p_snmp->index_data_req);
	RS485_M_Ctr = 0U; //RS485 receive mode

	memset(rx1_buff, 0, sizeof(rx1_buff));
	R_Config_SCI1_Serial_Receive((uint8_t*) &rx1_buff, sizeof(rx1_buff) - 1);
	R_BSP_SoftwareDelay(delay, BSP_DELAY_MILLISECS);

	memset(&p_snmp_var_out, 0, sizeof(&p_snmp_var_out));
	err_snmp = snmp_rx(rx1_buff, g_sci1_rx_count, oid_string,community_str_set, p_snmp_var_out.var_, &p_snmp_var_out.var_len,&p_snmp_var_out.type_var_);

	err_snmp = snmp_convert_snmp_var_2_var(&p_snmp_var_out,int_var_out,string_var_out,string_var_out_len,type_var);
	if(err_snmp != NO_ERR ) return err_snmp;
	if(type_var == INTEGER){

		if((*int_var_out)!= int_var_in){
			err_snmp = ERR;
			return err_snmp;
		}
	}else if(type_var == STRING){

		if (strcmp(string_var_out,string_var_in)!=0){
			err_snmp = ERR;
		    return err_snmp;
		}


	}
	return err_snmp;
}

///7.================================================================================================================================================
static uint8_t snmp_convert_var_2_snmp_var(int32_t  int_var, char *string_var, SNMP_VAR_T *p_snmp_var,uint8_t type_var  ){

	uint8_t err_snmp = NO_ERR;

	if(type_var == INTEGER)
	{
		if(int_var == NULL ) return ERR;

		memset(p_snmp_var, 0, sizeof(p_snmp_var));
		p_snmp_var->type_var_ = INTEGER;

		p_snmp_var->var_[0] = (uint8_t)(int_var>>24);
		p_snmp_var->var_[1] = (uint8_t)(int_var>>16);
		p_snmp_var->var_[2] = (uint8_t)(int_var>>8);
		p_snmp_var->var_[3] = (uint8_t)(int_var>>0);

		p_snmp_var->var_len = 4;

	}else if(type_var == STRING)
	{
		if(string_var == NULL ) return ERR;

		memset(p_snmp_var, 0, sizeof(p_snmp_var));
		p_snmp_var->type_var_ = STRING;

		for(uint8_t i = 0; i< strlen((const char *)string_var); i++){
			p_snmp_var->var_[i] = string_var[i];
		}
		p_snmp_var->var_len = strlen((const char *)string_var);
	}

	return err_snmp;
}
///8.================================================================================================================================================
static uint8_t snmp_convert_snmp_var_2_var(SNMP_VAR_T *p_snmp_var, int32_t  *int_var, char *string_var,uint8_t *string_var_len,  uint8_t type_var){
	uint8_t err_snmp = NO_ERR;
	int32_t vale_temp;
	uint8_t buff[4];

	if(type_var == INTEGER){
		if(int_var == NULL)                                       return ERR;
		if((p_snmp_var->var_len == 0)||(p_snmp_var->var_len > 4) )return ERR;
		vale_temp =0;

		if((p_snmp_var->var_[0]&0x80) == 0x80){
			for(uint8_t i = p_snmp_var->var_len; i<4; i++ ){
				buff[3-i] = 0xFF;
			}
			for(uint8_t i = 0; i<p_snmp_var->var_len; i++){
				buff[4 +i-p_snmp_var->var_len] = p_snmp_var->var_[i];
			}
			vale_temp = (buff[0]<<24)|(buff[1]<<16)|(buff[2]<<8)|(buff[3]<<0);
			*int_var = (int32_t)vale_temp;
		}else{
			for(uint8_t i =0; i <p_snmp_var->var_len; i++ ){
				vale_temp = vale_temp + (p_snmp_var->var_[i]<<(8*(p_snmp_var->var_len -1 -i)));
			}
			*int_var = (int32_t)vale_temp;
		}

	}else if(type_var == STRING){
	   if((string_var == NULL)||(string_var_len == NULL)) return ERR;
	   if((p_snmp_var->var_len == 0) )                    return ERR;

	   for(uint8_t i =0;i < p_snmp_var->var_len; i++ ){
		   string_var[i] = p_snmp_var->var_[i];
	   }
	   string_var[p_snmp_var->var_len] = '\0';
	   *string_var_len = p_snmp_var->var_len;
	}

	return err_snmp;
}
///9.=====================================================================================

uint8_t snmp_get_var_to_reg(SNMP_MANAGER_T *p_snmp, char *oid_string,
		char *community_str, int32_t *int_var_out, char *string_var_out,
		uint8_t *string_var_out_len, uint8_t type_var, uint16_t delay,
		uint16_t *reg, uint8_t *cnt_filter) {
	uint8_t err_snmp = NO_ERR;
	(*int_var_out) = 0;
	uint16_t u16_temp;
	err_snmp = snmp_get_var(p_snmp, oid_string, community_str, int_var_out, string_var_out, string_var_out_len, type_var, delay);

	if (err_snmp == NO_ERR) {
		if ((*int_var_out) == 0) {
			if ((*cnt_filter)++ > 15) {
				u16_temp = (uint16_t ) (*int_var_out);
				(*reg) = u16_temp;
				(*cnt_filter) = 0;
			}
		} else {
			if (strcmp(oid_string,TOTAL_RECT_DC_POWER) == 0) {
				u16_temp = (uint16_t) ((*int_var_out) / 1000);
			} else {
				u16_temp = (uint16_t ) (*int_var_out);
			}

			(*reg) = u16_temp;
			(*cnt_filter) = 0;
		}
	} else {
		(*reg) = 0xFFFF;
	}
	return err_snmp;
}

///10.handler snmp=========================================================================
int32_t  var_snmp_out;
extern uint16_t MDC_regs[125];
uint16_t err_cnt =0;
uint16_t no_err_cnt=0;
uint8_t  cnt_filter[60];

uint8_t handler_snmp(void){
	static uint8_t step =0;
	uint8_t err_snmp;
        switch(step++){
        case 0:
        	snmp_get_var_to_reg(&snmp_manager_t, BATT1_VOLT, COMMUNITY_STRING,
        			&var_snmp_out, NULL, NULL, INTEGER, DELAY_MS, &MDC_regs[0], &cnt_filter[0]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t, BATT2_VOLT, COMMUNITY_STRING,
        			&var_snmp_out, NULL, NULL, INTEGER, DELAY_MS, &MDC_regs[1], &cnt_filter[1]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t, BATT3_VOLT, COMMUNITY_STRING,
        			&var_snmp_out, NULL, NULL, INTEGER, DELAY_MS, &MDC_regs[2], &cnt_filter[2]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	err_snmp = snmp_get_var_to_reg(&snmp_manager_t, SYS_DC_VOLT, COMMUNITY_STRING,
        			&var_snmp_out, NULL, NULL, INTEGER, DELAY_MS, &MDC_regs[3], &cnt_filter[3]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	if (err_snmp == NO_ERR) {
        		no_err_cnt++;
        	} else {
        		err_cnt++;
        	}

        	snmp_get_var_to_reg(&snmp_manager_t,BATT1_CURR,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[5],&cnt_filter[5]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,BATT2_CURR,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[6],&cnt_filter[6]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,BATT3_CURR,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[7],&cnt_filter[7]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,DC_LOAD,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[8],&cnt_filter[8]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,SYS_TEMP,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[10],&cnt_filter[10]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,POWER_1,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[12],&cnt_filter[12]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);

        	//snmp_get_var_to_reg(&snmp_manager_t,OID_STRING_1,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[52]);
/*
        	err_snmp = snmp_get_var(&snmp_manager_t, OID_STRING_1, COMMUNITY_STRING, &var_snmp_out, NULL, NULL, INTEGER, 20);
			//err_snmp=snmp_get_var_to_reg(&snmp_manager_t,SYS_DC_VOLT,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,100,&mdc_reg[3]);
			if (err_snmp == NO_ERR) {
				mdc_reg[3] = (uint16_t) var_snmp_out;
				no_err_cnt++;
			} else {
				mdc_reg[3] = 0xffff;
				err_cnt++;
			}
        	//
*/
        	break;
        case 1:
        	snmp_get_var_to_reg(&snmp_manager_t,POWER_2,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[13],&cnt_filter[13]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
    	    snmp_get_var_to_reg(&snmp_manager_t,POWER_3,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[14],&cnt_filter[14]);
    	    //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
    	    snmp_get_var_to_reg(&snmp_manager_t,POWER_4,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[15],&cnt_filter[15]);
    	    //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
    	    snmp_get_var_to_reg(&snmp_manager_t,POWER_5,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[16],&cnt_filter[16]);
    	    //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
    	    snmp_get_var_to_reg(&snmp_manager_t,TOTAL_RECT_CURR,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[18],&cnt_filter[18]);
    	    //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
    	    snmp_get_var_to_reg(&snmp_manager_t,TOTAL_RECT_DC_POWER,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[19],&cnt_filter[19]);
    	    //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
    	    snmp_get_var_to_reg(&snmp_manager_t,RECT_STATUS,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[20],&cnt_filter[20]);
    	    //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
    	    snmp_get_var_to_reg(&snmp_manager_t,TIME_CHARGE_ACU1,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[21],&cnt_filter[21]);
    	    //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
    	    snmp_get_var_to_reg(&snmp_manager_t,TIME_DISCHARGE_ACU1,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[22],&cnt_filter[22]);
    	    //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
    	    snmp_get_var_to_reg(&snmp_manager_t,SOC1,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[23],&cnt_filter[23]);
    	    //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	break;
        case 2:
        	snmp_get_var_to_reg(&snmp_manager_t,SOH1,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[24],&cnt_filter[24]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,SATTUS_ACU1,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[25],&cnt_filter[25]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,TEMP_ACU1,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[26],&cnt_filter[26]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,SOC2,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[28],&cnt_filter[28]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,SOH2,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[29],&cnt_filter[29]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,SATTUS_ACU2,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[30],&cnt_filter[30]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,TEMP_ACU2,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[31],&cnt_filter[31]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,SOC3,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[33],&cnt_filter[33]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,SOH3,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[34],&cnt_filter[34]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,SATTUS_ACU3,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[35],&cnt_filter[35]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            break;
        case 3:
            snmp_get_var_to_reg(&snmp_manager_t,TEMP_ACU3,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[36],&cnt_filter[36]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,SOC4,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[38],&cnt_filter[38]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,SOH4,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[39],&cnt_filter[39]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,SATTUS_ACU4,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[40],&cnt_filter[40]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
        	snmp_get_var_to_reg(&snmp_manager_t,TEMP_ACU4,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[41],&cnt_filter[41]);
        	//R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,AC_VOLT,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[48],&cnt_filter[48]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,AC_CURR,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[49],&cnt_filter[49]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,BAT1_CAP,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[50],&cnt_filter[50]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,BAT2_CAP,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[51],&cnt_filter[51]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            snmp_get_var_to_reg(&snmp_manager_t,BAT3_CAP,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[52],&cnt_filter[52]);
            //R_BSP_SoftwareDelay(5, BSP_DELAY_MILLISECS);
            //snmp_get_var_to_reg(&snmp_manager_t,OID_STRING_1,COMMUNITY_STRING,&var_snmp_out,NULL,NULL,INTEGER,DELAY_MS,&MDC_regs[52]);

            break;
        default:
        	step =0;
        	break;
        }
	return NO_ERR;
}

























