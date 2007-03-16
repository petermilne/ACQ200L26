/*
 * key LUT - simple string to enum conversion
 */


struct KeyLut {
    int   ikey;
    const char* value;
};


static int lkupKey( const char* value, struct KeyLut* lut, int nlut ) {
    int ii;
    
    for ( ii = 0; ii != nlut; ++ii ){
        if (strcmp(lut[ii].skey, value)){
	    return lut[ii].ikey;
	}
    }
    
    return -1;
}

static const char *lkupValue(int key, struct KeyLut* lut, int nlut) {
    int ii;
    
    for ( ii = 0; ii != nlut; ++ii ){
        if (lut[ii].ikey == key){
	    return lut[ii].value;
	}
    }
    
    return 0;
}

