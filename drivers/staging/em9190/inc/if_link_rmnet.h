#ifndef _IF_LINK_RMNET_H
#define _IF_LINK_RMNET_H

/* rmnet section */

#define RMNET_FLAGS_INGRESS_DEAGGREGATION         (1U << 0)
#define RMNET_FLAGS_INGRESS_MAP_COMMANDS          (1U << 1)
#define RMNET_FLAGS_INGRESS_MAP_CKSUMV4           (1U << 2)
#define RMNET_FLAGS_EGRESS_MAP_CKSUMV4            (1U << 3)
#define RMNET_FLAGS_INGRESS_COALESCE              (1U << 4)
#define RMNET_FLAGS_INGRESS_MAP_CKSUMV5           (1U << 5)
#define RMNET_FLAGS_EGRESS_MAP_CKSUMV5            (1U << 6)

#endif
