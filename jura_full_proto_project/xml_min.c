#include "xml_min.h"
#include <string.h>
#include <stdio.h>

static const char* find_tag(const char *s, const char *tag, const char **end_tag){
    char open[64], close[64];
    snprintf(open, sizeof open, "<%s>", tag);
    snprintf(close,sizeof close,"</%s>", tag);
    const char *a = strstr(s, open); if (!a) return NULL;
    const char *b = strstr(a+strlen(open), close); if (!b) return NULL;
    *end_tag = b;
    return a + strlen(open);
}
static const char* walk_path(const char *xml, const char *path, const char **end){
    char tmp[96]; strncpy(tmp, path, sizeof tmp-1); tmp[sizeof tmp-1]=0;
    char *seg = strtok(tmp, "/");
    const char *cur = xml, *e=NULL;
    while (seg){ cur = find_tag(cur, seg, &e); if (!cur) return NULL; seg=strtok(NULL,"/"); }
    *end = e; return cur;
}
bool xml_get_value(const char *xml, const char *path, char *out, int out_max){
    const char *e, *v = walk_path(xml, path, &e); if (!v) return false;
    int len = (int)(e - v); if (len >= out_max) len = out_max-1;
    memcpy(out, v, len); out[len]=0; return true;
}
bool xml_set_value(char *xml, const char *path, const char *val){
    const char *e; char *start = (char*)walk_path(xml, path, &e); if (!start) return false;
    int old_len = (int)(e - start);
    int new_len = (int)strlen(val);
    if (new_len<=old_len){ memcpy(start, val, new_len); memmove(start+new_len, e, strlen(e)+1); }
    else { memcpy(start, val, old_len); }
    return true;
}