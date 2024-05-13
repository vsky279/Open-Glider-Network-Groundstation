#ifdef __cplusplus
extern "C"{
#endif

void crc_setup( void );

void            init_crc_ccitt_tab( void );
unsigned short update_crc_ccitt_16( unsigned short crc, unsigned char c );
void            update_crc8(unsigned char *crc, unsigned char m);

void            init_crc_bec_tab( unsigned short PacketSize );
unsigned short bec_correct_errors( unsigned short crc , unsigned char *pkt );

#ifdef __cplusplus
}
#endif

