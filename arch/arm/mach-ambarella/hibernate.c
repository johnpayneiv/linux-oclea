
/*
 * Hibernation support specific for ARM
 *
 * Derived from work on ARM hibernation support by:
 *
 * Ubuntu project, hibernation support for mach-dove
 * Copyright (C) 2010 Nokia Corporation (Hiroshi Doyu)
 * Copyright (C) 2010 Texas Instruments, Inc. (Teerth Reddy et al.)
 *  https://lkml.org/lkml/2010/6/18/4
 *  https://lists.linux-foundation.org/pipermail/linux-pm/2010-June/027422.html
 *  https://patchwork.kernel.org/patch/96442/
 *
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 *
 * Copyright (C) 2015 Ambarella, Shanghai(Jorney Tu)
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/mm.h>
#include <linux/suspend.h>
#include <asm/system_misc.h>
#include <asm/suspend.h>
#include <asm/memory.h>
#include <asm/sections.h>
#include <linux/mtd/mtd.h>
#include <linux/of.h>
#include <linux/lzo.h>
#include <linux/crc32.h>

#define HIBERNATE_MTD_NAME	"swp"
#define LZO_HEADER		sizeof(size_t)
#define LZO_UNC_PAGES		32
#define LZO_UNC_SIZE		(LZO_UNC_PAGES * PAGE_SIZE)

#define LZO_CMP_PAGES		DIV_ROUND_UP(lzo1x_worst_compress(LZO_UNC_SIZE) + \
					LZO_HEADER, PAGE_SIZE)
#define LZO_CMP_SIZE		(LZO_CMP_PAGES * PAGE_SIZE)

static int mtd_page_offset = 0;
static int hibernate_lzo_enable = 0;
static int hibernate_crc32_enable = 1;

static const unsigned int crc32_tab[] = {
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
        0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
        0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
        0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
        0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
        0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
        0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
        0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
        0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
        0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
        0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
        0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
        0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
        0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
        0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
        0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
        0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
        0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
        0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
        0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
        0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
        0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
        0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
        0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
        0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
        0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
        0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
        0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
        0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
        0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
        0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
        0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
        0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
        0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
        0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
        0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
        0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
        0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
        0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
        0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
        0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
        0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
        0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static unsigned int __crc32(unsigned int crc, const void *buf, unsigned int size)
{
        const unsigned char *p;
	unsigned int __crc = crc;

	/* if crc32 disable, return 0 */
	if (!hibernate_crc32_enable)
		return crc;

        p = buf;

        while (size > 0) {
                __crc = crc32_tab[(__crc ^ *p++) & 0xff] ^ (__crc >> 8);
		size--;
	}

        return __crc ^ ~0U;
}

static struct mtd_info *mtd_probe_dev(void)
{
	struct mtd_info *info = NULL;
	info = get_mtd_device_nm(HIBERNATE_MTD_NAME);

	if (IS_ERR(info)) {
		printk("SWP: mtd dev no found!\n");
		return NULL;
	} else {
		/* Makesure the swp partition has 32M at least */
		if(info->size < 0x2000000){
			printk("ERR: swp partition size is less than 32M\n");
			return NULL;
		}

		printk("MTD name:	%s\n",		info->name);
		printk("MTD size:	0x%llx\n",	info->size);
		printk("MTD blocksize:	0x%x\n",	info->erasesize);
		printk("MTD pagesize:	0x%x\n",	info->writesize);
	}
	return info;
}


static int hibernate_mtd_check(struct mtd_info *mtd, int ofs)
{

	int loff = ofs;
	int block = ofs / mtd->erasesize;

	while (mtd_block_isbad(mtd, loff) > 0){

		if(loff > mtd->size){
			printk("SWP: overflow mtd device ...\n");
			loff = 0;
			break;
		}

		printk("SWP: of.%08x or b.%d is a bad block\n", loff, block);
		block ++;
		loff += mtd->erasesize;
	}

	return loff / PAGE_SIZE;
}


static int hibernate_write_page(struct mtd_info *mtd, void *buf)
{

	int ret;
	size_t retlen;
	int offset = 0;

	/* Default: The 1st 4k(one PAGE_SIZE) is empty in "swp" mtd partition */
	mtd_page_offset++;

#if 1 /* bad block checking is needed ? */
	offset = hibernate_mtd_check(mtd, mtd_page_offset * PAGE_SIZE);
#else
	offset = mtd_page_offset;
#endif

	if(offset == 0)
		return -EINVAL;

	ret = mtd_write(mtd, PAGE_SIZE * offset, PAGE_SIZE, &retlen, buf);
	if(ret < 0){
		printk("SWP: MTD write failed!\n");
		return -EFAULT;
	}

	mtd_page_offset = offset;
	return 0;
}

static int hibernate_save_image(struct swsusp_info *header,
		struct mtd_info *mtd,
		struct snapshot_handle *snapshot)
{

	int ret;
	int nr_pages = 0;
	unsigned int crc = 0;

	while (1) {
		ret = snapshot_read_next(snapshot);
		if (ret <= 0)
			break;

		ret = hibernate_write_page(mtd, data_of(*snapshot));
		if (ret) {
			printk("Hibernation: write page error.\n");
			return ret;
		}

		nr_pages ++;

		/* Just generate the CRC of kernel snapshot , meta pages not included */
		if (nr_pages > nr_meta_pages) {
			crc = __crc32(crc, data_of(*snapshot), PAGE_SIZE);
		}
	}

	if(!ret)
		printk("LINUX:	%d pages, crc = %08x.\n", nr_pages - nr_meta_pages, crc);

	if(!nr_pages)
		ret = -EINVAL;

	header->crc32 = crc;
	header->lzo_enable = 0;

	/* save the header */
	mtd_page_offset = 0;
	hibernate_write_page(mtd, header);

	return ret;
}

static int hibernate_save_meta_pages(struct mtd_info *mtd,
		struct snapshot_handle *snapshot, int meta_pages)
{

	int ret;
	int nr_pages = 0;

	do {
		ret = snapshot_read_next(snapshot);
		if (ret <= 0)
			break;

		ret = hibernate_write_page(mtd, data_of(*snapshot));
		if (ret)
			break;

		nr_pages++;
	} while (nr_pages < meta_pages);

	BUG_ON(nr_pages != meta_pages);

	printk("SWP: %d meta pages have been saved!\n",  nr_pages);

	return ret;
}
#if 0
static void __lzo1x_decompress_safe(unsigned char *in, size_t len)
{
	size_t out_len = LZO_UNC_SIZE;
	unsigned char *out;

	out = vmalloc(LZO_UNC_SIZE);
	lzo1x_decompress_safe(in, len, out, &out_len);

	printk("__lzo1x_decompress_safe %08x - %08x\n", len, out_len);
	vfree(out);
}
#endif
static int hibernate_save_image_lzo(struct swsusp_info *header,
		struct mtd_info *mtd,
		struct snapshot_handle *snapshot)
{
	unsigned char *in, *out, *work_area;
	size_t out_len, offset = 0;
	int ret, nr_pages_unc = 0, nr_pages_lzo = 0;
	unsigned int unc_crc = 0, cmp_crc = 0;
#ifdef LZO_DEBUG
	int offset_debug, count = 0;
#endif

	in = vmalloc(LZO_UNC_SIZE);
	out = vmalloc(LZO_CMP_SIZE);
	work_area = vmalloc(LZO_UNC_SIZE);


	while (1) {
		for (offset = 0; offset < LZO_UNC_SIZE; offset += PAGE_SIZE) {
			ret = snapshot_read_next(snapshot);
			if (ret < 0)
				goto lzo_exception;

			if (!ret)
				break;

			memcpy(in + offset, data_of(*snapshot), PAGE_SIZE);
			nr_pages_unc++;
		}
		if (!offset)
			goto lzo_finish;

		unc_crc = __crc32(unc_crc, in, offset);

		lzo1x_1_compress(in, offset, out + LZO_HEADER, &out_len,
				work_area);

		if (!out_len || out_len > lzo1x_worst_compress(LZO_UNC_SIZE)) {
			printk("ERR: Invalid LZO compress length!\n");
			goto lzo_exception;
		}

		*(size_t *)out = out_len;

		cmp_crc = __crc32(cmp_crc, out + LZO_HEADER, out_len);

#ifdef LZO_DEBUG
		offset_debug = mtd_page_offset + 1;
#endif

		for (offset = 0; offset < out_len + LZO_HEADER; offset += PAGE_SIZE) {
			hibernate_write_page(mtd, out + offset);
			nr_pages_lzo++;
		}
#ifdef LZO_DEBUG
		count ++;
		printk("[LZO]%d:%d: %08x[%08x] compress to %08x [%08x]\n",
				offset_debug, count, LZO_UNC_SIZE, unc_crc,
				out_len, cmp_crc);
#endif

		//__lzo1x_decompress_safe(out + LZO_HEADER, out_len);
	}

lzo_finish:
	header->crc32 = unc_crc;
	header->lzo_enable = hibernate_lzo_enable;

	/* save the header */
	mtd_page_offset = 0;
	hibernate_write_page(mtd, header);

	printk("LINUX:	%d pages, crc = %08x\n", nr_pages_unc, unc_crc);
	printk("LZO:	%d pages, crc = %08x\n", nr_pages_lzo, cmp_crc);
lzo_exception:
	vfree(in);
	vfree(out);
	vfree(work_area);

	return 0;

}

static int hibernate_mtd_write(struct mtd_info *mtd)
{

	int error = 0;
	struct swsusp_info *header, *copy;
	struct snapshot_handle snapshot;

	memset(&snapshot, 0, sizeof(struct snapshot_handle));

	copy = vmalloc(sizeof(struct swsusp_info));
	if (!copy)
		return -ENOMEM;

	if(nr_meta_pages <= 0)
		return -EFAULT;

	error = snapshot_read_next(&snapshot);
	if (error < PAGE_SIZE) {
		if (error >= 0)
			error = -EFAULT;
		goto out_finish;
	}
	header = (struct swsusp_info *)data_of(snapshot);

	/* TODO: SWP partition space size check */
	if (!hibernate_lzo_enable && header->pages * 0x1000 > mtd->size) {
		printk("ERR: swp partition[0x%llx] has not enough space for the "
				"kernel snapshot[0x%lx]\n", mtd->size, header->pages * 0x1000);
			return -ENOMEM;
	}

	memcpy(copy, header, sizeof(struct swsusp_info));

#ifdef LZO_DEBUG
	printk("[LZO] pages:		%ld\n", header->pages);
	printk("[LZO] image_pages:	%ld\n", header->image_pages);
	printk("[LZO] nr_meta_pages:	%ld\n", header->pages - header->image_pages - 1);
#endif

	/*
	 * Skip saving the header. the header is copied and
	 * will be save after the CRC of the kernel snapshot
	 * has been generated. The mtd_page_offset should be
	 * set at meta data offset.
	 *
	 * */
	mtd_page_offset ++;

	printk("%s: save the linux snapshot\n",
			hibernate_lzo_enable ? "LZO" : "ORG");

	if (hibernate_lzo_enable) {
		hibernate_save_meta_pages(mtd, &snapshot, nr_meta_pages);
		error = hibernate_save_image_lzo(copy , mtd, &snapshot);
	} else {
		error = hibernate_save_image(copy, mtd, &snapshot);
	}

	vfree(copy);

out_finish:
	return error;

}
static void hibernate_of_parse(void)
{
	hibernate_lzo_enable = !!of_property_read_bool(of_chosen,
			"ambarella,hibernate-lzo-enable");
#if 0 /* FIXME */
	hibernate_crc32_enable = !!of_property_read_bool(of_chosen,
			"ambarella,hibernate-crc32-enable");
#endif
}

int swsusp_write_mtd(int flags)
{

	struct mtd_info *info = NULL;

	mtd_page_offset = 0;

	hibernate_of_parse();
	info = mtd_probe_dev();
	if (!info)
		return -EFAULT;

	return hibernate_mtd_write(info);
}

