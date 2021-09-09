/*
 * Copyright 2021 NXP
 * All rights reserved.
 * 
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hab.h"

/* Get CSF Header length */
#define GET_HAB_HEADER_LEN(hdr) (((hdr)->len[0] << 8) + (hdr)->len[1])

static int verify_ivt_header(struct ivt_header *p_ivt)
{
	if (p_ivt->tag != HAB_TAG_IVT) {
        PRINTF("bad tag 0x%x\r\n", p_ivt->tag);
		return true;
    }

	if ((uint8_t)(p_ivt->length >> 8) != IVT_TOTAL_LENGTH) {
        PRINTF("bad length 0x%02x\r\n", p_ivt->length);
		return true;
    }

	return false;
}

/* Get size of each CSF command sequence */
static int get_csf_cmd_hdr_len(uint8_t *csf_header)
{
    return (*csf_header == HAB_TAG_CSF) ? sizeof(struct hab_hdr) : 
                    GET_HAB_HEADER_LEN((struct hab_hdr *)csf_header);
}

/* Verify CSF command */
static bool verify_csf_command(uint8_t *csf_hdr, long start_addr, size_t bytes)
{
	uint8_t *start = (uint8_t *)start_addr;
	uint8_t *end;
	size_t csf_hdr_len;
	size_t cmd_hdr_len = 0;

	/* Verify if CSF pointer */
	if (csf_hdr == NULL) {
		PRINTF("Error: CSF pointer is null\n");
		return false;
	}

	/* Verify if CSF Header exist */
	if (*csf_hdr != HAB_TAG_CSF) {
		PRINTF("Error: No CSF header command\n");
		return false;
	}

	if (bytes != 0)
		end = start + bytes - 1;
	else
		end = start;

	csf_hdr_len = GET_HAB_HEADER_LEN((struct hab_hdr *)csf_hdr);

	/* Check if the CSF command is within the image bounds */
    if (!(csf_hdr && (csf_hdr >= start) && (csf_hdr <= end) &&
		((size_t)((end + 1) - csf_hdr) >= csf_hdr_len))) {
		PRINTF("Error: CSF command is outside the image bounds\n");
		return false;
	}

    for (size_t offset = 0; offset < csf_hdr_len; offset += cmd_hdr_len)
    {
		cmd_hdr_len = get_csf_cmd_hdr_len(&csf_hdr[offset]);
		if (!cmd_hdr_len) {
			PRINTF("Error: invalid command length\n");
			return false;
		}
    }

	return true;
}

int imx_hab_authenticate_image(uint32_t image_start, uint32_t image_size,
			       uint32_t ivt_offset)
{
	uint32_t load_addr = 0;
	size_t bytes = 0;

	uint32_t ivt_addr = image_start + ivt_offset;
	struct _ivt_ *ivt = (struct _ivt_ *)ivt_addr;

	enum hab_status status;
#if defined(CONFIG_HAB_CLOSE)
    enum hab_config config = HAB_CFG_FAB;
    enum hab_state state = HAB_STATE_NONE;
#endif

	PRINTF("\nAuthenticate image from location 0x%x...\n", image_start);

	/* Verify IVT header */
	if (verify_ivt_header(&ivt->hdr))
		goto hab_exit;

	/* Verify IVT pointer */
	if (ivt->self != ivt_addr) {
		PRINTF("Error: ivt->self = 0x%08x, ivt_addr = 0x%08x\n", ivt->self, ivt_addr);
		goto hab_exit;
	}

	/* Verify IVT DCD pointer */
	if (ivt->dcd) {
		PRINTF("Error: DCD pointer is not null\n");
		goto hab_exit;
	}

	bytes = ((BOOT_DATA_T *)(ivt->boot_data))->size;

	/* Verify CSF */
	if (!verify_csf_command((uint8_t *)ivt->csf, image_start, bytes))
		goto hab_exit;

	if (hab_Entry() != HAB_SUCCESS) {
		PRINTF("Error: hab entry function\n");
		goto hab_exit;
	}

	status = hab_rvt_check_target(HAB_TGT_MEMORY, (void *)image_start, bytes);
	if (status != HAB_SUCCESS) {
		PRINTF("HAB check target status %x, image_start 0x%08x, end 0x%08x\n", 
               status, image_start, image_start + bytes);
		goto hab_exit;
	}

	load_addr = (uint32_t)hab_Authenticate_image(1, ivt_offset, image_start,
			                                     bytes);

    status = hab_exit();
#if defined(CONFIG_HAB_CLOSE)
    status = hab_rvt_report_status(&config, &state);
#endif

    if ((status != HAB_FAILURE) 
#if defined(CONFIG_HAB_CLOSE)
        && (config == HAB_CFG_CLOSED) && (state == HAB_STATE_TRUSTED)
#endif
       )
    {
        //launch Application according the return by hab_authenticate_image
    }
    else
    {
        // boot failed
		PRINTF("hab verify fail\n");
		load_addr = 0;
    }

hab_exit:
	if (load_addr != 0)
		return false;

	return true;
}
