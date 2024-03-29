/*******************************************************************************
 Source file for the Net Pres Certificate Store functions to work with Harmony


  Summary:


  Description:

*******************************************************************************/

/*******************************************************************************
File Name: net_pres_cert_stroe.c
Copyright (c) 2015 released Microchip Technology Inc.  All rights
reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
#include "net/pres/net_pres_certstore.h"

// ISRG Root X1
// -----BEGIN CERTIFICATE-----
// MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
// TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
// cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
// WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
// ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
// MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
// h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
// 0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
// A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
// T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
// B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
// B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
// KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
// OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
// jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
// qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
// rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
// HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
// hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
// ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
// 3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
// NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
// ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
// TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
// jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
// oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
// 4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
// mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
// emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
// -----END CERTIFICATE-----
// openssl x509 -in "ISRG Root X1" -inform PEM -out cert.der -outform DER
// srec_cat cert.der -binary -output cert.c -c-Array ISRG_Root_X1 -include
const unsigned char ISRG_Root_X1[] =
{
    0x30, 0x82, 0x05, 0x6B, 0x30, 0x82, 0x03, 0x53, 0xA0, 0x03, 0x02, 0x01,
    0x02, 0x02, 0x11, 0x00, 0x82, 0x10, 0xCF, 0xB0, 0xD2, 0x40, 0xE3, 0x59,
    0x44, 0x63, 0xE0, 0xBB, 0x63, 0x82, 0x8B, 0x00, 0x30, 0x0D, 0x06, 0x09,
    0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x0B, 0x05, 0x00, 0x30,
    0x4F, 0x31, 0x0B, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02,
    0x55, 0x53, 0x31, 0x29, 0x30, 0x27, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x13,
    0x20, 0x49, 0x6E, 0x74, 0x65, 0x72, 0x6E, 0x65, 0x74, 0x20, 0x53, 0x65,
    0x63, 0x75, 0x72, 0x69, 0x74, 0x79, 0x20, 0x52, 0x65, 0x73, 0x65, 0x61,
    0x72, 0x63, 0x68, 0x20, 0x47, 0x72, 0x6F, 0x75, 0x70, 0x31, 0x15, 0x30,
    0x13, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x0C, 0x49, 0x53, 0x52, 0x47,
    0x20, 0x52, 0x6F, 0x6F, 0x74, 0x20, 0x58, 0x31, 0x30, 0x1E, 0x17, 0x0D,
    0x31, 0x35, 0x30, 0x36, 0x30, 0x34, 0x31, 0x31, 0x30, 0x34, 0x33, 0x38,
    0x5A, 0x17, 0x0D, 0x33, 0x35, 0x30, 0x36, 0x30, 0x34, 0x31, 0x31, 0x30,
    0x34, 0x33, 0x38, 0x5A, 0x30, 0x4F, 0x31, 0x0B, 0x30, 0x09, 0x06, 0x03,
    0x55, 0x04, 0x06, 0x13, 0x02, 0x55, 0x53, 0x31, 0x29, 0x30, 0x27, 0x06,
    0x03, 0x55, 0x04, 0x0A, 0x13, 0x20, 0x49, 0x6E, 0x74, 0x65, 0x72, 0x6E,
    0x65, 0x74, 0x20, 0x53, 0x65, 0x63, 0x75, 0x72, 0x69, 0x74, 0x79, 0x20,
    0x52, 0x65, 0x73, 0x65, 0x61, 0x72, 0x63, 0x68, 0x20, 0x47, 0x72, 0x6F,
    0x75, 0x70, 0x31, 0x15, 0x30, 0x13, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13,
    0x0C, 0x49, 0x53, 0x52, 0x47, 0x20, 0x52, 0x6F, 0x6F, 0x74, 0x20, 0x58,
    0x31, 0x30, 0x82, 0x02, 0x22, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48,
    0x86, 0xF7, 0x0D, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x02, 0x0F,
    0x00, 0x30, 0x82, 0x02, 0x0A, 0x02, 0x82, 0x02, 0x01, 0x00, 0xAD, 0xE8,
    0x24, 0x73, 0xF4, 0x14, 0x37, 0xF3, 0x9B, 0x9E, 0x2B, 0x57, 0x28, 0x1C,
    0x87, 0xBE, 0xDC, 0xB7, 0xDF, 0x38, 0x90, 0x8C, 0x6E, 0x3C, 0xE6, 0x57,
    0xA0, 0x78, 0xF7, 0x75, 0xC2, 0xA2, 0xFE, 0xF5, 0x6A, 0x6E, 0xF6, 0x00,
    0x4F, 0x28, 0xDB, 0xDE, 0x68, 0x86, 0x6C, 0x44, 0x93, 0xB6, 0xB1, 0x63,
    0xFD, 0x14, 0x12, 0x6B, 0xBF, 0x1F, 0xD2, 0xEA, 0x31, 0x9B, 0x21, 0x7E,
    0xD1, 0x33, 0x3C, 0xBA, 0x48, 0xF5, 0xDD, 0x79, 0xDF, 0xB3, 0xB8, 0xFF,
    0x12, 0xF1, 0x21, 0x9A, 0x4B, 0xC1, 0x8A, 0x86, 0x71, 0x69, 0x4A, 0x66,
    0x66, 0x6C, 0x8F, 0x7E, 0x3C, 0x70, 0xBF, 0xAD, 0x29, 0x22, 0x06, 0xF3,
    0xE4, 0xC0, 0xE6, 0x80, 0xAE, 0xE2, 0x4B, 0x8F, 0xB7, 0x99, 0x7E, 0x94,
    0x03, 0x9F, 0xD3, 0x47, 0x97, 0x7C, 0x99, 0x48, 0x23, 0x53, 0xE8, 0x38,
    0xAE, 0x4F, 0x0A, 0x6F, 0x83, 0x2E, 0xD1, 0x49, 0x57, 0x8C, 0x80, 0x74,
    0xB6, 0xDA, 0x2F, 0xD0, 0x38, 0x8D, 0x7B, 0x03, 0x70, 0x21, 0x1B, 0x75,
    0xF2, 0x30, 0x3C, 0xFA, 0x8F, 0xAE, 0xDD, 0xDA, 0x63, 0xAB, 0xEB, 0x16,
    0x4F, 0xC2, 0x8E, 0x11, 0x4B, 0x7E, 0xCF, 0x0B, 0xE8, 0xFF, 0xB5, 0x77,
    0x2E, 0xF4, 0xB2, 0x7B, 0x4A, 0xE0, 0x4C, 0x12, 0x25, 0x0C, 0x70, 0x8D,
    0x03, 0x29, 0xA0, 0xE1, 0x53, 0x24, 0xEC, 0x13, 0xD9, 0xEE, 0x19, 0xBF,
    0x10, 0xB3, 0x4A, 0x8C, 0x3F, 0x89, 0xA3, 0x61, 0x51, 0xDE, 0xAC, 0x87,
    0x07, 0x94, 0xF4, 0x63, 0x71, 0xEC, 0x2E, 0xE2, 0x6F, 0x5B, 0x98, 0x81,
    0xE1, 0x89, 0x5C, 0x34, 0x79, 0x6C, 0x76, 0xEF, 0x3B, 0x90, 0x62, 0x79,
    0xE6, 0xDB, 0xA4, 0x9A, 0x2F, 0x26, 0xC5, 0xD0, 0x10, 0xE1, 0x0E, 0xDE,
    0xD9, 0x10, 0x8E, 0x16, 0xFB, 0xB7, 0xF7, 0xA8, 0xF7, 0xC7, 0xE5, 0x02,
    0x07, 0x98, 0x8F, 0x36, 0x08, 0x95, 0xE7, 0xE2, 0x37, 0x96, 0x0D, 0x36,
    0x75, 0x9E, 0xFB, 0x0E, 0x72, 0xB1, 0x1D, 0x9B, 0xBC, 0x03, 0xF9, 0x49,
    0x05, 0xD8, 0x81, 0xDD, 0x05, 0xB4, 0x2A, 0xD6, 0x41, 0xE9, 0xAC, 0x01,
    0x76, 0x95, 0x0A, 0x0F, 0xD8, 0xDF, 0xD5, 0xBD, 0x12, 0x1F, 0x35, 0x2F,
    0x28, 0x17, 0x6C, 0xD2, 0x98, 0xC1, 0xA8, 0x09, 0x64, 0x77, 0x6E, 0x47,
    0x37, 0xBA, 0xCE, 0xAC, 0x59, 0x5E, 0x68, 0x9D, 0x7F, 0x72, 0xD6, 0x89,
    0xC5, 0x06, 0x41, 0x29, 0x3E, 0x59, 0x3E, 0xDD, 0x26, 0xF5, 0x24, 0xC9,
    0x11, 0xA7, 0x5A, 0xA3, 0x4C, 0x40, 0x1F, 0x46, 0xA1, 0x99, 0xB5, 0xA7,
    0x3A, 0x51, 0x6E, 0x86, 0x3B, 0x9E, 0x7D, 0x72, 0xA7, 0x12, 0x05, 0x78,
    0x59, 0xED, 0x3E, 0x51, 0x78, 0x15, 0x0B, 0x03, 0x8F, 0x8D, 0xD0, 0x2F,
    0x05, 0xB2, 0x3E, 0x7B, 0x4A, 0x1C, 0x4B, 0x73, 0x05, 0x12, 0xFC, 0xC6,
    0xEA, 0xE0, 0x50, 0x13, 0x7C, 0x43, 0x93, 0x74, 0xB3, 0xCA, 0x74, 0xE7,
    0x8E, 0x1F, 0x01, 0x08, 0xD0, 0x30, 0xD4, 0x5B, 0x71, 0x36, 0xB4, 0x07,
    0xBA, 0xC1, 0x30, 0x30, 0x5C, 0x48, 0xB7, 0x82, 0x3B, 0x98, 0xA6, 0x7D,
    0x60, 0x8A, 0xA2, 0xA3, 0x29, 0x82, 0xCC, 0xBA, 0xBD, 0x83, 0x04, 0x1B,
    0xA2, 0x83, 0x03, 0x41, 0xA1, 0xD6, 0x05, 0xF1, 0x1B, 0xC2, 0xB6, 0xF0,
    0xA8, 0x7C, 0x86, 0x3B, 0x46, 0xA8, 0x48, 0x2A, 0x88, 0xDC, 0x76, 0x9A,
    0x76, 0xBF, 0x1F, 0x6A, 0xA5, 0x3D, 0x19, 0x8F, 0xEB, 0x38, 0xF3, 0x64,
    0xDE, 0xC8, 0x2B, 0x0D, 0x0A, 0x28, 0xFF, 0xF7, 0xDB, 0xE2, 0x15, 0x42,
    0xD4, 0x22, 0xD0, 0x27, 0x5D, 0xE1, 0x79, 0xFE, 0x18, 0xE7, 0x70, 0x88,
    0xAD, 0x4E, 0xE6, 0xD9, 0x8B, 0x3A, 0xC6, 0xDD, 0x27, 0x51, 0x6E, 0xFF,
    0xBC, 0x64, 0xF5, 0x33, 0x43, 0x4F, 0x02, 0x03, 0x01, 0x00, 0x01, 0xA3,
    0x42, 0x30, 0x40, 0x30, 0x0E, 0x06, 0x03, 0x55, 0x1D, 0x0F, 0x01, 0x01,
    0xFF, 0x04, 0x04, 0x03, 0x02, 0x01, 0x06, 0x30, 0x0F, 0x06, 0x03, 0x55,
    0x1D, 0x13, 0x01, 0x01, 0xFF, 0x04, 0x05, 0x30, 0x03, 0x01, 0x01, 0xFF,
    0x30, 0x1D, 0x06, 0x03, 0x55, 0x1D, 0x0E, 0x04, 0x16, 0x04, 0x14, 0x79,
    0xB4, 0x59, 0xE6, 0x7B, 0xB6, 0xE5, 0xE4, 0x01, 0x73, 0x80, 0x08, 0x88,
    0xC8, 0x1A, 0x58, 0xF6, 0xE9, 0x9B, 0x6E, 0x30, 0x0D, 0x06, 0x09, 0x2A,
    0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x0B, 0x05, 0x00, 0x03, 0x82,
    0x02, 0x01, 0x00, 0x55, 0x1F, 0x58, 0xA9, 0xBC, 0xB2, 0xA8, 0x50, 0xD0,
    0x0C, 0xB1, 0xD8, 0x1A, 0x69, 0x20, 0x27, 0x29, 0x08, 0xAC, 0x61, 0x75,
    0x5C, 0x8A, 0x6E, 0xF8, 0x82, 0xE5, 0x69, 0x2F, 0xD5, 0xF6, 0x56, 0x4B,
    0xB9, 0xB8, 0x73, 0x10, 0x59, 0xD3, 0x21, 0x97, 0x7E, 0xE7, 0x4C, 0x71,
    0xFB, 0xB2, 0xD2, 0x60, 0xAD, 0x39, 0xA8, 0x0B, 0xEA, 0x17, 0x21, 0x56,
    0x85, 0xF1, 0x50, 0x0E, 0x59, 0xEB, 0xCE, 0xE0, 0x59, 0xE9, 0xBA, 0xC9,
    0x15, 0xEF, 0x86, 0x9D, 0x8F, 0x84, 0x80, 0xF6, 0xE4, 0xE9, 0x91, 0x90,
    0xDC, 0x17, 0x9B, 0x62, 0x1B, 0x45, 0xF0, 0x66, 0x95, 0xD2, 0x7C, 0x6F,
    0xC2, 0xEA, 0x3B, 0xEF, 0x1F, 0xCF, 0xCB, 0xD6, 0xAE, 0x27, 0xF1, 0xA9,
    0xB0, 0xC8, 0xAE, 0xFD, 0x7D, 0x7E, 0x9A, 0xFA, 0x22, 0x04, 0xEB, 0xFF,
    0xD9, 0x7F, 0xEA, 0x91, 0x2B, 0x22, 0xB1, 0x17, 0x0E, 0x8F, 0xF2, 0x8A,
    0x34, 0x5B, 0x58, 0xD8, 0xFC, 0x01, 0xC9, 0x54, 0xB9, 0xB8, 0x26, 0xCC,
    0x8A, 0x88, 0x33, 0x89, 0x4C, 0x2D, 0x84, 0x3C, 0x82, 0xDF, 0xEE, 0x96,
    0x57, 0x05, 0xBA, 0x2C, 0xBB, 0xF7, 0xC4, 0xB7, 0xC7, 0x4E, 0x3B, 0x82,
    0xBE, 0x31, 0xC8, 0x22, 0x73, 0x73, 0x92, 0xD1, 0xC2, 0x80, 0xA4, 0x39,
    0x39, 0x10, 0x33, 0x23, 0x82, 0x4C, 0x3C, 0x9F, 0x86, 0xB2, 0x55, 0x98,
    0x1D, 0xBE, 0x29, 0x86, 0x8C, 0x22, 0x9B, 0x9E, 0xE2, 0x6B, 0x3B, 0x57,
    0x3A, 0x82, 0x70, 0x4D, 0xDC, 0x09, 0xC7, 0x89, 0xCB, 0x0A, 0x07, 0x4D,
    0x6C, 0xE8, 0x5D, 0x8E, 0xC9, 0xEF, 0xCE, 0xAB, 0xC7, 0xBB, 0xB5, 0x2B,
    0x4E, 0x45, 0xD6, 0x4A, 0xD0, 0x26, 0xCC, 0xE5, 0x72, 0xCA, 0x08, 0x6A,
    0xA5, 0x95, 0xE3, 0x15, 0xA1, 0xF7, 0xA4, 0xED, 0xC9, 0x2C, 0x5F, 0xA5,
    0xFB, 0xFF, 0xAC, 0x28, 0x02, 0x2E, 0xBE, 0xD7, 0x7B, 0xBB, 0xE3, 0x71,
    0x7B, 0x90, 0x16, 0xD3, 0x07, 0x5E, 0x46, 0x53, 0x7C, 0x37, 0x07, 0x42,
    0x8C, 0xD3, 0xC4, 0x96, 0x9C, 0xD5, 0x99, 0xB5, 0x2A, 0xE0, 0x95, 0x1A,
    0x80, 0x48, 0xAE, 0x4C, 0x39, 0x07, 0xCE, 0xCC, 0x47, 0xA4, 0x52, 0x95,
    0x2B, 0xBA, 0xB8, 0xFB, 0xAD, 0xD2, 0x33, 0x53, 0x7D, 0xE5, 0x1D, 0x4D,
    0x6D, 0xD5, 0xA1, 0xB1, 0xC7, 0x42, 0x6F, 0xE6, 0x40, 0x27, 0x35, 0x5C,
    0xA3, 0x28, 0xB7, 0x07, 0x8D, 0xE7, 0x8D, 0x33, 0x90, 0xE7, 0x23, 0x9F,
    0xFB, 0x50, 0x9C, 0x79, 0x6C, 0x46, 0xD5, 0xB4, 0x15, 0xB3, 0x96, 0x6E,
    0x7E, 0x9B, 0x0C, 0x96, 0x3A, 0xB8, 0x52, 0x2D, 0x3F, 0xD6, 0x5B, 0xE1,
    0xFB, 0x08, 0xC2, 0x84, 0xFE, 0x24, 0xA8, 0xA3, 0x89, 0xDA, 0xAC, 0x6A,
    0xE1, 0x18, 0x2A, 0xB1, 0xA8, 0x43, 0x61, 0x5B, 0xD3, 0x1F, 0xDC, 0x3B,
    0x8D, 0x76, 0xF2, 0x2D, 0xE8, 0x8D, 0x75, 0xDF, 0x17, 0x33, 0x6C, 0x3D,
    0x53, 0xFB, 0x7B, 0xCB, 0x41, 0x5F, 0xFF, 0xDC, 0xA2, 0xD0, 0x61, 0x38,
    0xE1, 0x96, 0xB8, 0xAC, 0x5D, 0x8B, 0x37, 0xD7, 0x75, 0xD5, 0x33, 0xC0,
    0x99, 0x11, 0xAE, 0x9D, 0x41, 0xC1, 0x72, 0x75, 0x84, 0xBE, 0x02, 0x41,
    0x42, 0x5F, 0x67, 0x24, 0x48, 0x94, 0xD1, 0x9B, 0x27, 0xBE, 0x07, 0x3F,
    0xB9, 0xB8, 0x4F, 0x81, 0x74, 0x51, 0xE1, 0x7A, 0xB7, 0xED, 0x9D, 0x23,
    0xE2, 0xBE, 0xE0, 0xD5, 0x28, 0x04, 0x13, 0x3C, 0x31, 0x03, 0x9E, 0xDD,
    0x7A, 0x6C, 0x8F, 0xC6, 0x07, 0x18, 0xC6, 0x7F, 0xDE, 0x47, 0x8E, 0x3F,
    0x28, 0x9E, 0x04, 0x06, 0xCF, 0xA5, 0x54, 0x34, 0x77, 0xBD, 0xEC, 0x89,
    0x9B, 0xE9, 0x17, 0x43, 0xDF, 0x5B, 0xDB, 0x5F, 0xFE, 0x8E, 0x1E, 0x57,
    0xA2, 0xCD, 0x40, 0x9D, 0x7E, 0x62, 0x22, 0xDA, 0xDE, 0x18, 0x27,
};

// DigiCert Global Root G2
// -----BEGIN CERTIFICATE-----
// MIIEizCCA3OgAwIBAgIQDI7gyQ1qiRWIBAYe4kH5rzANBgkqhkiG9w0BAQsFADBh
// MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
// d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
// MjAeFw0xMzA4MDExMjAwMDBaFw0yODA4MDExMjAwMDBaMEQxCzAJBgNVBAYTAlVT
// MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxHjAcBgNVBAMTFURpZ2lDZXJ0IEdsb2Jh
// bCBDQSBHMjCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBANNIfL7zBYZd
// W9UvhU5L4IatFaxhz1uvPmoKR/uadpFgC4przc/cV35gmAvkVNlW7SHMArZagV+X
// au4CLyMnuG3UsOcGAngLH1ypmTb+u6wbBfpXzYEQQGfWMItYNdSWYb7QjHqXnxr5
// IuYUL6nG6AEfq/gmD6yOTSwyOR2Bm40cZbIc22GoiS9g5+vCShjEbyrpEJIJ7RfR
// ACvmfe8EiRROM6GyD5eHn7OgzS+8LOy4g2gxPR/VSpAQGQuBldYpdlH5NnbQtwl6
// OErXb4y/E3w57bqukPyV93t4CTZedJMeJfD/1K2uaGvG/w/VNfFVbkhJ+Pi474j4
// 8V4Rd6rfArMCAwEAAaOCAVowggFWMBIGA1UdEwEB/wQIMAYBAf8CAQAwDgYDVR0P
// AQH/BAQDAgGGMDQGCCsGAQUFBwEBBCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29j
// c3AuZGlnaWNlcnQuY29tMHsGA1UdHwR0MHIwN6A1oDOGMWh0dHA6Ly9jcmw0LmRp
// Z2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RHMi5jcmwwN6A1oDOGMWh0dHA6
// Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEdsb2JhbFJvb3RHMi5jcmwwPQYD
// VR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEWHGh0dHBzOi8vd3d3LmRpZ2lj
// ZXJ0LmNvbS9DUFMwHQYDVR0OBBYEFCRuKy3QapJRUSVpAaqaR6aJ50AgMB8GA1Ud
// IwQYMBaAFE4iVCAYlebjbuYP+vq5Eu0GF485MA0GCSqGSIb3DQEBCwUAA4IBAQAL
// OYSR+ZfrqoGvhOlaOJL84mxZvzbIRacxAxHhBsCsMsdaVSnaT0AC9aHesO3ewPj2
// dZ12uYf+QYB6z13jAMZbAuabeGLJ3LhimnftiQjXS8X9Q9ViIyfEBFltcT8jW+rZ
// 8uckJ2/0lYDblizkVIvP6hnZf1WZUXoOLRg9eFhSvGNoVwvdRLNXSmDmyHBwW4co
// atc7TlJFGa8kBpJIERqLrqwYElesA8u49L3KJg6nwd3jM+/AVTANlVlOnAM2BvjA
// jxSZnE0qnsHhfTuvcqdFuhOWKU4Z0BqYBvQ3lBetoxi6PrABDJXWKTUgNX31EGDk
// 92hiHuwZ4STyhxGs6QiA
// -----END CERTIFICATE-----
// openssl x509 -in "DigiCert Global Root G2" -inform PEM -out cert.der -outform DER
// srec_cat cert.der -binary -output cert.c -c-Array DigiCert_Global_Root_G2 -include
const unsigned char DigiCert_Global_Root_G2[] =
{
    0x30, 0x82, 0x04, 0x8B, 0x30, 0x82, 0x03, 0x73, 0xA0, 0x03, 0x02, 0x01,
    0x02, 0x02, 0x10, 0x0C, 0x8E, 0xE0, 0xC9, 0x0D, 0x6A, 0x89, 0x15, 0x88,
    0x04, 0x06, 0x1E, 0xE2, 0x41, 0xF9, 0xAF, 0x30, 0x0D, 0x06, 0x09, 0x2A,
    0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x0B, 0x05, 0x00, 0x30, 0x61,
    0x31, 0x0B, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55,
    0x53, 0x31, 0x15, 0x30, 0x13, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x13, 0x0C,
    0x44, 0x69, 0x67, 0x69, 0x43, 0x65, 0x72, 0x74, 0x20, 0x49, 0x6E, 0x63,
    0x31, 0x19, 0x30, 0x17, 0x06, 0x03, 0x55, 0x04, 0x0B, 0x13, 0x10, 0x77,
    0x77, 0x77, 0x2E, 0x64, 0x69, 0x67, 0x69, 0x63, 0x65, 0x72, 0x74, 0x2E,
    0x63, 0x6F, 0x6D, 0x31, 0x20, 0x30, 0x1E, 0x06, 0x03, 0x55, 0x04, 0x03,
    0x13, 0x17, 0x44, 0x69, 0x67, 0x69, 0x43, 0x65, 0x72, 0x74, 0x20, 0x47,
    0x6C, 0x6F, 0x62, 0x61, 0x6C, 0x20, 0x52, 0x6F, 0x6F, 0x74, 0x20, 0x47,
    0x32, 0x30, 0x1E, 0x17, 0x0D, 0x31, 0x33, 0x30, 0x38, 0x30, 0x31, 0x31,
    0x32, 0x30, 0x30, 0x30, 0x30, 0x5A, 0x17, 0x0D, 0x32, 0x38, 0x30, 0x38,
    0x30, 0x31, 0x31, 0x32, 0x30, 0x30, 0x30, 0x30, 0x5A, 0x30, 0x44, 0x31,
    0x0B, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55, 0x53,
    0x31, 0x15, 0x30, 0x13, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x13, 0x0C, 0x44,
    0x69, 0x67, 0x69, 0x43, 0x65, 0x72, 0x74, 0x20, 0x49, 0x6E, 0x63, 0x31,
    0x1E, 0x30, 0x1C, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x15, 0x44, 0x69,
    0x67, 0x69, 0x43, 0x65, 0x72, 0x74, 0x20, 0x47, 0x6C, 0x6F, 0x62, 0x61,
    0x6C, 0x20, 0x43, 0x41, 0x20, 0x47, 0x32, 0x30, 0x82, 0x01, 0x22, 0x30,
    0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x01,
    0x05, 0x00, 0x03, 0x82, 0x01, 0x0F, 0x00, 0x30, 0x82, 0x01, 0x0A, 0x02,
    0x82, 0x01, 0x01, 0x00, 0xD3, 0x48, 0x7C, 0xBE, 0xF3, 0x05, 0x86, 0x5D,
    0x5B, 0xD5, 0x2F, 0x85, 0x4E, 0x4B, 0xE0, 0x86, 0xAD, 0x15, 0xAC, 0x61,
    0xCF, 0x5B, 0xAF, 0x3E, 0x6A, 0x0A, 0x47, 0xFB, 0x9A, 0x76, 0x91, 0x60,
    0x0B, 0x8A, 0x6B, 0xCD, 0xCF, 0xDC, 0x57, 0x7E, 0x60, 0x98, 0x0B, 0xE4,
    0x54, 0xD9, 0x56, 0xED, 0x21, 0xCC, 0x02, 0xB6, 0x5A, 0x81, 0x5F, 0x97,
    0x6A, 0xEE, 0x02, 0x2F, 0x23, 0x27, 0xB8, 0x6D, 0xD4, 0xB0, 0xE7, 0x06,
    0x02, 0x78, 0x0B, 0x1F, 0x5C, 0xA9, 0x99, 0x36, 0xFE, 0xBB, 0xAC, 0x1B,
    0x05, 0xFA, 0x57, 0xCD, 0x81, 0x10, 0x40, 0x67, 0xD6, 0x30, 0x8B, 0x58,
    0x35, 0xD4, 0x96, 0x61, 0xBE, 0xD0, 0x8C, 0x7A, 0x97, 0x9F, 0x1A, 0xF9,
    0x22, 0xE6, 0x14, 0x2F, 0xA9, 0xC6, 0xE8, 0x01, 0x1F, 0xAB, 0xF8, 0x26,
    0x0F, 0xAC, 0x8E, 0x4D, 0x2C, 0x32, 0x39, 0x1D, 0x81, 0x9B, 0x8D, 0x1C,
    0x65, 0xB2, 0x1C, 0xDB, 0x61, 0xA8, 0x89, 0x2F, 0x60, 0xE7, 0xEB, 0xC2,
    0x4A, 0x18, 0xC4, 0x6F, 0x2A, 0xE9, 0x10, 0x92, 0x09, 0xED, 0x17, 0xD1,
    0x00, 0x2B, 0xE6, 0x7D, 0xEF, 0x04, 0x89, 0x14, 0x4E, 0x33, 0xA1, 0xB2,
    0x0F, 0x97, 0x87, 0x9F, 0xB3, 0xA0, 0xCD, 0x2F, 0xBC, 0x2C, 0xEC, 0xB8,
    0x83, 0x68, 0x31, 0x3D, 0x1F, 0xD5, 0x4A, 0x90, 0x10, 0x19, 0x0B, 0x81,
    0x95, 0xD6, 0x29, 0x76, 0x51, 0xF9, 0x36, 0x76, 0xD0, 0xB7, 0x09, 0x7A,
    0x38, 0x4A, 0xD7, 0x6F, 0x8C, 0xBF, 0x13, 0x7C, 0x39, 0xED, 0xBA, 0xAE,
    0x90, 0xFC, 0x95, 0xF7, 0x7B, 0x78, 0x09, 0x36, 0x5E, 0x74, 0x93, 0x1E,
    0x25, 0xF0, 0xFF, 0xD4, 0xAD, 0xAE, 0x68, 0x6B, 0xC6, 0xFF, 0x0F, 0xD5,
    0x35, 0xF1, 0x55, 0x6E, 0x48, 0x49, 0xF8, 0xF8, 0xB8, 0xEF, 0x88, 0xF8,
    0xF1, 0x5E, 0x11, 0x77, 0xAA, 0xDF, 0x02, 0xB3, 0x02, 0x03, 0x01, 0x00,
    0x01, 0xA3, 0x82, 0x01, 0x5A, 0x30, 0x82, 0x01, 0x56, 0x30, 0x12, 0x06,
    0x03, 0x55, 0x1D, 0x13, 0x01, 0x01, 0xFF, 0x04, 0x08, 0x30, 0x06, 0x01,
    0x01, 0xFF, 0x02, 0x01, 0x00, 0x30, 0x0E, 0x06, 0x03, 0x55, 0x1D, 0x0F,
    0x01, 0x01, 0xFF, 0x04, 0x04, 0x03, 0x02, 0x01, 0x86, 0x30, 0x34, 0x06,
    0x08, 0x2B, 0x06, 0x01, 0x05, 0x05, 0x07, 0x01, 0x01, 0x04, 0x28, 0x30,
    0x26, 0x30, 0x24, 0x06, 0x08, 0x2B, 0x06, 0x01, 0x05, 0x05, 0x07, 0x30,
    0x01, 0x86, 0x18, 0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x6F, 0x63,
    0x73, 0x70, 0x2E, 0x64, 0x69, 0x67, 0x69, 0x63, 0x65, 0x72, 0x74, 0x2E,
    0x63, 0x6F, 0x6D, 0x30, 0x7B, 0x06, 0x03, 0x55, 0x1D, 0x1F, 0x04, 0x74,
    0x30, 0x72, 0x30, 0x37, 0xA0, 0x35, 0xA0, 0x33, 0x86, 0x31, 0x68, 0x74,
    0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x63, 0x72, 0x6C, 0x34, 0x2E, 0x64, 0x69,
    0x67, 0x69, 0x63, 0x65, 0x72, 0x74, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 0x44,
    0x69, 0x67, 0x69, 0x43, 0x65, 0x72, 0x74, 0x47, 0x6C, 0x6F, 0x62, 0x61,
    0x6C, 0x52, 0x6F, 0x6F, 0x74, 0x47, 0x32, 0x2E, 0x63, 0x72, 0x6C, 0x30,
    0x37, 0xA0, 0x35, 0xA0, 0x33, 0x86, 0x31, 0x68, 0x74, 0x74, 0x70, 0x3A,
    0x2F, 0x2F, 0x63, 0x72, 0x6C, 0x33, 0x2E, 0x64, 0x69, 0x67, 0x69, 0x63,
    0x65, 0x72, 0x74, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 0x44, 0x69, 0x67, 0x69,
    0x43, 0x65, 0x72, 0x74, 0x47, 0x6C, 0x6F, 0x62, 0x61, 0x6C, 0x52, 0x6F,
    0x6F, 0x74, 0x47, 0x32, 0x2E, 0x63, 0x72, 0x6C, 0x30, 0x3D, 0x06, 0x03,
    0x55, 0x1D, 0x20, 0x04, 0x36, 0x30, 0x34, 0x30, 0x32, 0x06, 0x04, 0x55,
    0x1D, 0x20, 0x00, 0x30, 0x2A, 0x30, 0x28, 0x06, 0x08, 0x2B, 0x06, 0x01,
    0x05, 0x05, 0x07, 0x02, 0x01, 0x16, 0x1C, 0x68, 0x74, 0x74, 0x70, 0x73,
    0x3A, 0x2F, 0x2F, 0x77, 0x77, 0x77, 0x2E, 0x64, 0x69, 0x67, 0x69, 0x63,
    0x65, 0x72, 0x74, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 0x43, 0x50, 0x53, 0x30,
    0x1D, 0x06, 0x03, 0x55, 0x1D, 0x0E, 0x04, 0x16, 0x04, 0x14, 0x24, 0x6E,
    0x2B, 0x2D, 0xD0, 0x6A, 0x92, 0x51, 0x51, 0x25, 0x69, 0x01, 0xAA, 0x9A,
    0x47, 0xA6, 0x89, 0xE7, 0x40, 0x20, 0x30, 0x1F, 0x06, 0x03, 0x55, 0x1D,
    0x23, 0x04, 0x18, 0x30, 0x16, 0x80, 0x14, 0x4E, 0x22, 0x54, 0x20, 0x18,
    0x95, 0xE6, 0xE3, 0x6E, 0xE6, 0x0F, 0xFA, 0xFA, 0xB9, 0x12, 0xED, 0x06,
    0x17, 0x8F, 0x39, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7,
    0x0D, 0x01, 0x01, 0x0B, 0x05, 0x00, 0x03, 0x82, 0x01, 0x01, 0x00, 0x0B,
    0x39, 0x84, 0x91, 0xF9, 0x97, 0xEB, 0xAA, 0x81, 0xAF, 0x84, 0xE9, 0x5A,
    0x38, 0x92, 0xFC, 0xE2, 0x6C, 0x59, 0xBF, 0x36, 0xC8, 0x45, 0xA7, 0x31,
    0x03, 0x11, 0xE1, 0x06, 0xC0, 0xAC, 0x32, 0xC7, 0x5A, 0x55, 0x29, 0xDA,
    0x4F, 0x40, 0x02, 0xF5, 0xA1, 0xDE, 0xB0, 0xED, 0xDE, 0xC0, 0xF8, 0xF6,
    0x75, 0x9D, 0x76, 0xB9, 0x87, 0xFE, 0x41, 0x80, 0x7A, 0xCF, 0x5D, 0xE3,
    0x00, 0xC6, 0x5B, 0x02, 0xE6, 0x9B, 0x78, 0x62, 0xC9, 0xDC, 0xB8, 0x62,
    0x9A, 0x77, 0xED, 0x89, 0x08, 0xD7, 0x4B, 0xC5, 0xFD, 0x43, 0xD5, 0x62,
    0x23, 0x27, 0xC4, 0x04, 0x59, 0x6D, 0x71, 0x3F, 0x23, 0x5B, 0xEA, 0xD9,
    0xF2, 0xE7, 0x24, 0x27, 0x6F, 0xF4, 0x95, 0x80, 0xDB, 0x96, 0x2C, 0xE4,
    0x54, 0x8B, 0xCF, 0xEA, 0x19, 0xD9, 0x7F, 0x55, 0x99, 0x51, 0x7A, 0x0E,
    0x2D, 0x18, 0x3D, 0x78, 0x58, 0x52, 0xBC, 0x63, 0x68, 0x57, 0x0B, 0xDD,
    0x44, 0xB3, 0x57, 0x4A, 0x60, 0xE6, 0xC8, 0x70, 0x70, 0x5B, 0x87, 0x28,
    0x6A, 0xD7, 0x3B, 0x4E, 0x52, 0x45, 0x19, 0xAF, 0x24, 0x06, 0x92, 0x48,
    0x11, 0x1A, 0x8B, 0xAE, 0xAC, 0x18, 0x12, 0x57, 0xAC, 0x03, 0xCB, 0xB8,
    0xF4, 0xBD, 0xCA, 0x26, 0x0E, 0xA7, 0xC1, 0xDD, 0xE3, 0x33, 0xEF, 0xC0,
    0x55, 0x30, 0x0D, 0x95, 0x59, 0x4E, 0x9C, 0x03, 0x36, 0x06, 0xF8, 0xC0,
    0x8F, 0x14, 0x99, 0x9C, 0x4D, 0x2A, 0x9E, 0xC1, 0xE1, 0x7D, 0x3B, 0xAF,
    0x72, 0xA7, 0x45, 0xBA, 0x13, 0x96, 0x29, 0x4E, 0x19, 0xD0, 0x1A, 0x98,
    0x06, 0xF4, 0x37, 0x94, 0x17, 0xAD, 0xA3, 0x18, 0xBA, 0x3E, 0xB0, 0x01,
    0x0C, 0x95, 0xD6, 0x29, 0x35, 0x20, 0x35, 0x7D, 0xF5, 0x10, 0x60, 0xE4,
    0xF7, 0x68, 0x62, 0x1E, 0xEC, 0x19, 0xE1, 0x24, 0xF2, 0x87, 0x11, 0xAC,
    0xE9, 0x08, 0x80,
};


// Baltimore CyberTrust Root
// -----BEGIN CERTIFICATE-----
// MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ
// RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD
// VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX
// DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y
// ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy
// VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr
// mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr
// IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK
// mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu
// XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy
// dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye
// jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1
// BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3
// DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92
// 9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx
// jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0
// Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz
// ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS
// R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp
// -----END CERTIFICATE-----
// openssl x509 -in "Baltimore CyberTrust Root" -inform PEM -out cert.der -outform DER
// srec_cat cert.der -binary -output cert.c -c-Array Baltimore_CyberTrust_Root -include
const unsigned char Baltimore_CyberTrust_Root[] =
{
    0x30, 0x82, 0x03, 0x77, 0x30, 0x82, 0x02, 0x5F, 0xA0, 0x03, 0x02, 0x01,
    0x02, 0x02, 0x04, 0x02, 0x00, 0x00, 0xB9, 0x30, 0x0D, 0x06, 0x09, 0x2A,
    0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x05, 0x05, 0x00, 0x30, 0x5A,
    0x31, 0x0B, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x49,
    0x45, 0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x13, 0x09,
    0x42, 0x61, 0x6C, 0x74, 0x69, 0x6D, 0x6F, 0x72, 0x65, 0x31, 0x13, 0x30,
    0x11, 0x06, 0x03, 0x55, 0x04, 0x0B, 0x13, 0x0A, 0x43, 0x79, 0x62, 0x65,
    0x72, 0x54, 0x72, 0x75, 0x73, 0x74, 0x31, 0x22, 0x30, 0x20, 0x06, 0x03,
    0x55, 0x04, 0x03, 0x13, 0x19, 0x42, 0x61, 0x6C, 0x74, 0x69, 0x6D, 0x6F,
    0x72, 0x65, 0x20, 0x43, 0x79, 0x62, 0x65, 0x72, 0x54, 0x72, 0x75, 0x73,
    0x74, 0x20, 0x52, 0x6F, 0x6F, 0x74, 0x30, 0x1E, 0x17, 0x0D, 0x30, 0x30,
    0x30, 0x35, 0x31, 0x32, 0x31, 0x38, 0x34, 0x36, 0x30, 0x30, 0x5A, 0x17,
    0x0D, 0x32, 0x35, 0x30, 0x35, 0x31, 0x32, 0x32, 0x33, 0x35, 0x39, 0x30,
    0x30, 0x5A, 0x30, 0x5A, 0x31, 0x0B, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04,
    0x06, 0x13, 0x02, 0x49, 0x45, 0x31, 0x12, 0x30, 0x10, 0x06, 0x03, 0x55,
    0x04, 0x0A, 0x13, 0x09, 0x42, 0x61, 0x6C, 0x74, 0x69, 0x6D, 0x6F, 0x72,
    0x65, 0x31, 0x13, 0x30, 0x11, 0x06, 0x03, 0x55, 0x04, 0x0B, 0x13, 0x0A,
    0x43, 0x79, 0x62, 0x65, 0x72, 0x54, 0x72, 0x75, 0x73, 0x74, 0x31, 0x22,
    0x30, 0x20, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x19, 0x42, 0x61, 0x6C,
    0x74, 0x69, 0x6D, 0x6F, 0x72, 0x65, 0x20, 0x43, 0x79, 0x62, 0x65, 0x72,
    0x54, 0x72, 0x75, 0x73, 0x74, 0x20, 0x52, 0x6F, 0x6F, 0x74, 0x30, 0x82,
    0x01, 0x22, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D,
    0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0F, 0x00, 0x30, 0x82,
    0x01, 0x0A, 0x02, 0x82, 0x01, 0x01, 0x00, 0xA3, 0x04, 0xBB, 0x22, 0xAB,
    0x98, 0x3D, 0x57, 0xE8, 0x26, 0x72, 0x9A, 0xB5, 0x79, 0xD4, 0x29, 0xE2,
    0xE1, 0xE8, 0x95, 0x80, 0xB1, 0xB0, 0xE3, 0x5B, 0x8E, 0x2B, 0x29, 0x9A,
    0x64, 0xDF, 0xA1, 0x5D, 0xED, 0xB0, 0x09, 0x05, 0x6D, 0xDB, 0x28, 0x2E,
    0xCE, 0x62, 0xA2, 0x62, 0xFE, 0xB4, 0x88, 0xDA, 0x12, 0xEB, 0x38, 0xEB,
    0x21, 0x9D, 0xC0, 0x41, 0x2B, 0x01, 0x52, 0x7B, 0x88, 0x77, 0xD3, 0x1C,
    0x8F, 0xC7, 0xBA, 0xB9, 0x88, 0xB5, 0x6A, 0x09, 0xE7, 0x73, 0xE8, 0x11,
    0x40, 0xA7, 0xD1, 0xCC, 0xCA, 0x62, 0x8D, 0x2D, 0xE5, 0x8F, 0x0B, 0xA6,
    0x50, 0xD2, 0xA8, 0x50, 0xC3, 0x28, 0xEA, 0xF5, 0xAB, 0x25, 0x87, 0x8A,
    0x9A, 0x96, 0x1C, 0xA9, 0x67, 0xB8, 0x3F, 0x0C, 0xD5, 0xF7, 0xF9, 0x52,
    0x13, 0x2F, 0xC2, 0x1B, 0xD5, 0x70, 0x70, 0xF0, 0x8F, 0xC0, 0x12, 0xCA,
    0x06, 0xCB, 0x9A, 0xE1, 0xD9, 0xCA, 0x33, 0x7A, 0x77, 0xD6, 0xF8, 0xEC,
    0xB9, 0xF1, 0x68, 0x44, 0x42, 0x48, 0x13, 0xD2, 0xC0, 0xC2, 0xA4, 0xAE,
    0x5E, 0x60, 0xFE, 0xB6, 0xA6, 0x05, 0xFC, 0xB4, 0xDD, 0x07, 0x59, 0x02,
    0xD4, 0x59, 0x18, 0x98, 0x63, 0xF5, 0xA5, 0x63, 0xE0, 0x90, 0x0C, 0x7D,
    0x5D, 0xB2, 0x06, 0x7A, 0xF3, 0x85, 0xEA, 0xEB, 0xD4, 0x03, 0xAE, 0x5E,
    0x84, 0x3E, 0x5F, 0xFF, 0x15, 0xED, 0x69, 0xBC, 0xF9, 0x39, 0x36, 0x72,
    0x75, 0xCF, 0x77, 0x52, 0x4D, 0xF3, 0xC9, 0x90, 0x2C, 0xB9, 0x3D, 0xE5,
    0xC9, 0x23, 0x53, 0x3F, 0x1F, 0x24, 0x98, 0x21, 0x5C, 0x07, 0x99, 0x29,
    0xBD, 0xC6, 0x3A, 0xEC, 0xE7, 0x6E, 0x86, 0x3A, 0x6B, 0x97, 0x74, 0x63,
    0x33, 0xBD, 0x68, 0x18, 0x31, 0xF0, 0x78, 0x8D, 0x76, 0xBF, 0xFC, 0x9E,
    0x8E, 0x5D, 0x2A, 0x86, 0xA7, 0x4D, 0x90, 0xDC, 0x27, 0x1A, 0x39, 0x02,
    0x03, 0x01, 0x00, 0x01, 0xA3, 0x45, 0x30, 0x43, 0x30, 0x1D, 0x06, 0x03,
    0x55, 0x1D, 0x0E, 0x04, 0x16, 0x04, 0x14, 0xE5, 0x9D, 0x59, 0x30, 0x82,
    0x47, 0x58, 0xCC, 0xAC, 0xFA, 0x08, 0x54, 0x36, 0x86, 0x7B, 0x3A, 0xB5,
    0x04, 0x4D, 0xF0, 0x30, 0x12, 0x06, 0x03, 0x55, 0x1D, 0x13, 0x01, 0x01,
    0xFF, 0x04, 0x08, 0x30, 0x06, 0x01, 0x01, 0xFF, 0x02, 0x01, 0x03, 0x30,
    0x0E, 0x06, 0x03, 0x55, 0x1D, 0x0F, 0x01, 0x01, 0xFF, 0x04, 0x04, 0x03,
    0x02, 0x01, 0x06, 0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7,
    0x0D, 0x01, 0x01, 0x05, 0x05, 0x00, 0x03, 0x82, 0x01, 0x01, 0x00, 0x85,
    0x0C, 0x5D, 0x8E, 0xE4, 0x6F, 0x51, 0x68, 0x42, 0x05, 0xA0, 0xDD, 0xBB,
    0x4F, 0x27, 0x25, 0x84, 0x03, 0xBD, 0xF7, 0x64, 0xFD, 0x2D, 0xD7, 0x30,
    0xE3, 0xA4, 0x10, 0x17, 0xEB, 0xDA, 0x29, 0x29, 0xB6, 0x79, 0x3F, 0x76,
    0xF6, 0x19, 0x13, 0x23, 0xB8, 0x10, 0x0A, 0xF9, 0x58, 0xA4, 0xD4, 0x61,
    0x70, 0xBD, 0x04, 0x61, 0x6A, 0x12, 0x8A, 0x17, 0xD5, 0x0A, 0xBD, 0xC5,
    0xBC, 0x30, 0x7C, 0xD6, 0xE9, 0x0C, 0x25, 0x8D, 0x86, 0x40, 0x4F, 0xEC,
    0xCC, 0xA3, 0x7E, 0x38, 0xC6, 0x37, 0x11, 0x4F, 0xED, 0xDD, 0x68, 0x31,
    0x8E, 0x4C, 0xD2, 0xB3, 0x01, 0x74, 0xEE, 0xBE, 0x75, 0x5E, 0x07, 0x48,
    0x1A, 0x7F, 0x70, 0xFF, 0x16, 0x5C, 0x84, 0xC0, 0x79, 0x85, 0xB8, 0x05,
    0xFD, 0x7F, 0xBE, 0x65, 0x11, 0xA3, 0x0F, 0xC0, 0x02, 0xB4, 0xF8, 0x52,
    0x37, 0x39, 0x04, 0xD5, 0xA9, 0x31, 0x7A, 0x18, 0xBF, 0xA0, 0x2A, 0xF4,
    0x12, 0x99, 0xF7, 0xA3, 0x45, 0x82, 0xE3, 0x3C, 0x5E, 0xF5, 0x9D, 0x9E,
    0xB5, 0xC8, 0x9E, 0x7C, 0x2E, 0xC8, 0xA4, 0x9E, 0x4E, 0x08, 0x14, 0x4B,
    0x6D, 0xFD, 0x70, 0x6D, 0x6B, 0x1A, 0x63, 0xBD, 0x64, 0xE6, 0x1F, 0xB7,
    0xCE, 0xF0, 0xF2, 0x9F, 0x2E, 0xBB, 0x1B, 0xB7, 0xF2, 0x50, 0x88, 0x73,
    0x92, 0xC2, 0xE2, 0xE3, 0x16, 0x8D, 0x9A, 0x32, 0x02, 0xAB, 0x8E, 0x18,
    0xDD, 0xE9, 0x10, 0x11, 0xEE, 0x7E, 0x35, 0xAB, 0x90, 0xAF, 0x3E, 0x30,
    0x94, 0x7A, 0xD0, 0x33, 0x3D, 0xA7, 0x65, 0x0F, 0xF5, 0xFC, 0x8E, 0x9E,
    0x62, 0xCF, 0x47, 0x44, 0x2C, 0x01, 0x5D, 0xBB, 0x1D, 0xB5, 0x32, 0xD2,
    0x47, 0xD2, 0x38, 0x2E, 0xD0, 0xFE, 0x81, 0xDC, 0x32, 0x6A, 0x1E, 0xB5,
    0xEE, 0x3C, 0xD5, 0xFC, 0xE7, 0x81, 0x1D, 0x19, 0xC3, 0x24, 0x42, 0xEA,
    0x63, 0x39, 0xA9,
};

// Amazon Root CA 1
// -----BEGIN CERTIFICATE-----
// MIIEkjCCA3qgAwIBAgITBn+USionzfP6wq4rAfkI7rnExjANBgkqhkiG9w0BAQsF
// ADCBmDELMAkGA1UEBhMCVVMxEDAOBgNVBAgTB0FyaXpvbmExEzARBgNVBAcTClNj
// b3R0c2RhbGUxJTAjBgNVBAoTHFN0YXJmaWVsZCBUZWNobm9sb2dpZXMsIEluYy4x
// OzA5BgNVBAMTMlN0YXJmaWVsZCBTZXJ2aWNlcyBSb290IENlcnRpZmljYXRlIEF1
// dGhvcml0eSAtIEcyMB4XDTE1MDUyNTEyMDAwMFoXDTM3MTIzMTAxMDAwMFowOTEL
// MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
// b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
// ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
// 9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
// IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
// VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
// 93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
// jgSubJrIqg0CAwEAAaOCATEwggEtMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/
// BAQDAgGGMB0GA1UdDgQWBBSEGMyFNOy8DJSULghZnMeyEE4KCDAfBgNVHSMEGDAW
// gBScXwDfqgHXMCs4iKK4bUqc8hGRgzB4BggrBgEFBQcBAQRsMGowLgYIKwYBBQUH
// MAGGImh0dHA6Ly9vY3NwLnJvb3RnMi5hbWF6b250cnVzdC5jb20wOAYIKwYBBQUH
// MAKGLGh0dHA6Ly9jcnQucm9vdGcyLmFtYXpvbnRydXN0LmNvbS9yb290ZzIuY2Vy
// MD0GA1UdHwQ2MDQwMqAwoC6GLGh0dHA6Ly9jcmwucm9vdGcyLmFtYXpvbnRydXN0
// LmNvbS9yb290ZzIuY3JsMBEGA1UdIAQKMAgwBgYEVR0gADANBgkqhkiG9w0BAQsF
// AAOCAQEAYjdCXLwQtT6LLOkMm2xF4gcAevnFWAu5CIw+7bMlPLVvUOTNNWqnkzSW
// MiGpSESrnO09tKpzbeR/FoCJbM8oAxiDR3mjEH4wW6w7sGDgd9QIpuEdfF7Au/ma
// eyKdpwAJfqxGF4PcnCZXmTA5YpaP7dreqsXMGz7KQ2hsVxa81Q4gLv7/wmpdLqBK
// bRRYh5TmOTFffHPLkIhqhBGWJ6bt2YFGpn6jcgAKUj6DiAdjd4lpFw85hdKrCEVN
// 0FE6/V1dN2RMfjCyVSRCnTawXZwXgWHxyvkQAiSr6w10kY17RSlQOYiypok1JR4U
// akcjMS9cmvqtmg5iUaQqqcT5NJ0hGA==
// -----END CERTIFICATE-----
// openssl x509 -in <AMAZON_ROOT_CA_1.pem> -inform PEM -out cert.der -outform DER
// srec_cat cert.der -binary -output cert.c -c-Array AMAZON_ROOT_CA_1 -include
const unsigned char AMAZON_ROOT_CA_1[] =
{
    0x30, 0x82, 0x04, 0x49, 0x30, 0x82, 0x03, 0x31, 0xA0, 0x03, 0x02, 0x01,
    0x02, 0x02, 0x13, 0x06, 0x7F, 0x94, 0x57, 0x85, 0x87, 0xE8, 0xAC, 0x77,
    0xDE, 0xB2, 0x53, 0x32, 0x5B, 0xBC, 0x99, 0x8B, 0x56, 0x0D, 0x30, 0x0D,
    0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x0B, 0x05,
    0x00, 0x30, 0x39, 0x31, 0x0B, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06,
    0x13, 0x02, 0x55, 0x53, 0x31, 0x0F, 0x30, 0x0D, 0x06, 0x03, 0x55, 0x04,
    0x0A, 0x13, 0x06, 0x41, 0x6D, 0x61, 0x7A, 0x6F, 0x6E, 0x31, 0x19, 0x30,
    0x17, 0x06, 0x03, 0x55, 0x04, 0x03, 0x13, 0x10, 0x41, 0x6D, 0x61, 0x7A,
    0x6F, 0x6E, 0x20, 0x52, 0x6F, 0x6F, 0x74, 0x20, 0x43, 0x41, 0x20, 0x31,
    0x30, 0x1E, 0x17, 0x0D, 0x31, 0x35, 0x31, 0x30, 0x32, 0x32, 0x30, 0x30,
    0x30, 0x30, 0x30, 0x30, 0x5A, 0x17, 0x0D, 0x32, 0x35, 0x31, 0x30, 0x31,
    0x39, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x5A, 0x30, 0x46, 0x31, 0x0B,
    0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 0x55, 0x53, 0x31,
    0x0F, 0x30, 0x0D, 0x06, 0x03, 0x55, 0x04, 0x0A, 0x13, 0x06, 0x41, 0x6D,
    0x61, 0x7A, 0x6F, 0x6E, 0x31, 0x15, 0x30, 0x13, 0x06, 0x03, 0x55, 0x04,
    0x0B, 0x13, 0x0C, 0x53, 0x65, 0x72, 0x76, 0x65, 0x72, 0x20, 0x43, 0x41,
    0x20, 0x31, 0x42, 0x31, 0x0F, 0x30, 0x0D, 0x06, 0x03, 0x55, 0x04, 0x03,
    0x13, 0x06, 0x41, 0x6D, 0x61, 0x7A, 0x6F, 0x6E, 0x30, 0x82, 0x01, 0x22,
    0x30, 0x0D, 0x06, 0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01,
    0x01, 0x05, 0x00, 0x03, 0x82, 0x01, 0x0F, 0x00, 0x30, 0x82, 0x01, 0x0A,
    0x02, 0x82, 0x01, 0x01, 0x00, 0xC2, 0x4E, 0x16, 0x67, 0xDD, 0xCE, 0xBC,
    0x6A, 0xC8, 0x37, 0x5A, 0xEC, 0x3A, 0x30, 0xB0, 0x1D, 0xE6, 0xD1, 0x12,
    0xE8, 0x12, 0x28, 0x48, 0xCC, 0xE8, 0x29, 0xC1, 0xB9, 0x6E, 0x53, 0xD5,
    0xA3, 0xEB, 0x03, 0x39, 0x1A, 0xCC, 0x77, 0x87, 0xF6, 0x01, 0xB9, 0xD9,
    0x70, 0xCC, 0xCF, 0x6B, 0x8D, 0xE3, 0xE3, 0x03, 0x71, 0x86, 0x99, 0x6D,
    0xCB, 0xA6, 0x94, 0x2A, 0x4E, 0x13, 0xD6, 0xA7, 0xBD, 0x04, 0xEC, 0x0A,
    0x16, 0x3C, 0x0A, 0xEB, 0x39, 0xB1, 0xC4, 0xB5, 0x58, 0xA3, 0xB6, 0xC7,
    0x56, 0x25, 0xEC, 0x3E, 0x52, 0x7A, 0xA8, 0xE3, 0x29, 0x16, 0x07, 0xB9,
    0x6E, 0x50, 0xCF, 0xFB, 0x5F, 0x31, 0xF8, 0x1D, 0xBA, 0x03, 0x4A, 0x62,
    0x89, 0x03, 0xAE, 0x3E, 0x47, 0xF2, 0x0F, 0x27, 0x91, 0xE3, 0x14, 0x20,
    0x85, 0xF8, 0xFA, 0xE9, 0x8A, 0x35, 0xF5, 0x5F, 0x9E, 0x99, 0x4D, 0xE7,
    0x6B, 0x37, 0xEF, 0xA4, 0x50, 0x3E, 0x44, 0xEC, 0xFA, 0x5A, 0x85, 0x66,
    0x07, 0x9C, 0x7E, 0x17, 0x6A, 0x55, 0xF3, 0x17, 0x8A, 0x35, 0x1E, 0xEE,
    0xE9, 0xAC, 0xC3, 0x75, 0x4E, 0x58, 0x55, 0x7D, 0x53, 0x6B, 0x0A, 0x6B,
    0x9B, 0x14, 0x42, 0xD7, 0xE5, 0xAC, 0x01, 0x89, 0xB3, 0xEA, 0xA3, 0xFE,
    0xCF, 0xC0, 0x2B, 0x0C, 0x84, 0xC2, 0xD8, 0x53, 0x15, 0xCB, 0x67, 0xF0,
    0xD0, 0x88, 0xCA, 0x3A, 0xD1, 0x17, 0x73, 0xF5, 0x5F, 0x9A, 0xD4, 0xC5,
    0x72, 0x1E, 0x7E, 0x01, 0xF1, 0x98, 0x30, 0x63, 0x2A, 0xAA, 0xF2, 0x7A,
    0x2D, 0xC5, 0xE2, 0x02, 0x1A, 0x86, 0xE5, 0x32, 0x3E, 0x0E, 0xBD, 0x11,
    0xB4, 0xCF, 0x3C, 0x93, 0xEF, 0x17, 0x50, 0x10, 0x9E, 0x43, 0xC2, 0x06,
    0x2A, 0xE0, 0x0D, 0x68, 0xBE, 0xD3, 0x88, 0x8B, 0x4A, 0x65, 0x8C, 0x4A,
    0xD4, 0xC3, 0x2E, 0x4C, 0x9B, 0x55, 0xF4, 0x86, 0xE5, 0x02, 0x03, 0x01,
    0x00, 0x01, 0xA3, 0x82, 0x01, 0x3B, 0x30, 0x82, 0x01, 0x37, 0x30, 0x12,
    0x06, 0x03, 0x55, 0x1D, 0x13, 0x01, 0x01, 0xFF, 0x04, 0x08, 0x30, 0x06,
    0x01, 0x01, 0xFF, 0x02, 0x01, 0x00, 0x30, 0x0E, 0x06, 0x03, 0x55, 0x1D,
    0x0F, 0x01, 0x01, 0xFF, 0x04, 0x04, 0x03, 0x02, 0x01, 0x86, 0x30, 0x1D,
    0x06, 0x03, 0x55, 0x1D, 0x0E, 0x04, 0x16, 0x04, 0x14, 0x59, 0xA4, 0x66,
    0x06, 0x52, 0xA0, 0x7B, 0x95, 0x92, 0x3C, 0xA3, 0x94, 0x07, 0x27, 0x96,
    0x74, 0x5B, 0xF9, 0x3D, 0xD0, 0x30, 0x1F, 0x06, 0x03, 0x55, 0x1D, 0x23,
    0x04, 0x18, 0x30, 0x16, 0x80, 0x14, 0x84, 0x18, 0xCC, 0x85, 0x34, 0xEC,
    0xBC, 0x0C, 0x94, 0x94, 0x2E, 0x08, 0x59, 0x9C, 0xC7, 0xB2, 0x10, 0x4E,
    0x0A, 0x08, 0x30, 0x7B, 0x06, 0x08, 0x2B, 0x06, 0x01, 0x05, 0x05, 0x07,
    0x01, 0x01, 0x04, 0x6F, 0x30, 0x6D, 0x30, 0x2F, 0x06, 0x08, 0x2B, 0x06,
    0x01, 0x05, 0x05, 0x07, 0x30, 0x01, 0x86, 0x23, 0x68, 0x74, 0x74, 0x70,
    0x3A, 0x2F, 0x2F, 0x6F, 0x63, 0x73, 0x70, 0x2E, 0x72, 0x6F, 0x6F, 0x74,
    0x63, 0x61, 0x31, 0x2E, 0x61, 0x6D, 0x61, 0x7A, 0x6F, 0x6E, 0x74, 0x72,
    0x75, 0x73, 0x74, 0x2E, 0x63, 0x6F, 0x6D, 0x30, 0x3A, 0x06, 0x08, 0x2B,
    0x06, 0x01, 0x05, 0x05, 0x07, 0x30, 0x02, 0x86, 0x2E, 0x68, 0x74, 0x74,
    0x70, 0x3A, 0x2F, 0x2F, 0x63, 0x72, 0x74, 0x2E, 0x72, 0x6F, 0x6F, 0x74,
    0x63, 0x61, 0x31, 0x2E, 0x61, 0x6D, 0x61, 0x7A, 0x6F, 0x6E, 0x74, 0x72,
    0x75, 0x73, 0x74, 0x2E, 0x63, 0x6F, 0x6D, 0x2F, 0x72, 0x6F, 0x6F, 0x74,
    0x63, 0x61, 0x31, 0x2E, 0x63, 0x65, 0x72, 0x30, 0x3F, 0x06, 0x03, 0x55,
    0x1D, 0x1F, 0x04, 0x38, 0x30, 0x36, 0x30, 0x34, 0xA0, 0x32, 0xA0, 0x30,
    0x86, 0x2E, 0x68, 0x74, 0x74, 0x70, 0x3A, 0x2F, 0x2F, 0x63, 0x72, 0x6C,
    0x2E, 0x72, 0x6F, 0x6F, 0x74, 0x63, 0x61, 0x31, 0x2E, 0x61, 0x6D, 0x61,
    0x7A, 0x6F, 0x6E, 0x74, 0x72, 0x75, 0x73, 0x74, 0x2E, 0x63, 0x6F, 0x6D,
    0x2F, 0x72, 0x6F, 0x6F, 0x74, 0x63, 0x61, 0x31, 0x2E, 0x63, 0x72, 0x6C,
    0x30, 0x13, 0x06, 0x03, 0x55, 0x1D, 0x20, 0x04, 0x0C, 0x30, 0x0A, 0x30,
    0x08, 0x06, 0x06, 0x67, 0x81, 0x0C, 0x01, 0x02, 0x01, 0x30, 0x0D, 0x06,
    0x09, 0x2A, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x01, 0x01, 0x0B, 0x05, 0x00,
    0x03, 0x82, 0x01, 0x01, 0x00, 0x85, 0x92, 0xBE, 0x35, 0xBB, 0x79, 0xCF,
    0xA3, 0x81, 0x42, 0x1C, 0xE4, 0xE3, 0x63, 0x73, 0x53, 0x39, 0x52, 0x35,
    0xE7, 0xD1, 0xAD, 0xFD, 0xAE, 0x99, 0x8A, 0xAC, 0x89, 0x12, 0x2F, 0xBB,
    0xE7, 0x6F, 0x9A, 0xD5, 0x4E, 0x72, 0xEA, 0x20, 0x30, 0x61, 0xF9, 0x97,
    0xB2, 0xCD, 0xA5, 0x27, 0x02, 0x45, 0xA8, 0xCA, 0x76, 0x3E, 0x98, 0x4A,
    0x83, 0x9E, 0xB6, 0xE6, 0x45, 0xE0, 0xF2, 0x43, 0xF6, 0x08, 0xDE, 0x6D,
    0xE8, 0x6E, 0xDB, 0x31, 0x07, 0x13, 0xF0, 0x2F, 0x31, 0x0D, 0x93, 0x6D,
    0x61, 0x37, 0x7B, 0x58, 0xF0, 0xFC, 0x51, 0x98, 0x91, 0x28, 0x02, 0x4F,
    0x05, 0x76, 0xB7, 0xD3, 0xF0, 0x1B, 0xC2, 0xE6, 0x5E, 0xD0, 0x66, 0x85,
    0x11, 0x0F, 0x2E, 0x81, 0xC6, 0x10, 0x81, 0x29, 0xFE, 0x20, 0x60, 0x48,
    0xF3, 0xF2, 0xF0, 0x84, 0x13, 0x53, 0x65, 0x35, 0x15, 0x11, 0x6B, 0x82,
    0x51, 0x40, 0x55, 0x57, 0x5F, 0x18, 0xB5, 0xB0, 0x22, 0x3E, 0xAD, 0xF2,
    0x5E, 0xA3, 0x01, 0xE3, 0xC3, 0xB3, 0xF9, 0xCB, 0x41, 0x5A, 0xE6, 0x52,
    0x91, 0xBB, 0xE4, 0x36, 0x87, 0x4F, 0x2D, 0xA9, 0xA4, 0x07, 0x68, 0x35,
    0xBA, 0x94, 0x72, 0xCD, 0x0E, 0xEA, 0x0E, 0x7D, 0x57, 0xF2, 0x79, 0xFC,
    0x37, 0xC5, 0x7B, 0x60, 0x9E, 0xB2, 0xEB, 0xC0, 0x2D, 0x90, 0x77, 0x0D,
    0x49, 0x10, 0x27, 0xA5, 0x38, 0xAD, 0xC4, 0x12, 0xA3, 0xB4, 0xA3, 0xC8,
    0x48, 0xB3, 0x15, 0x0B, 0x1E, 0xE2, 0xE2, 0x19, 0xDC, 0xC4, 0x76, 0x52,
    0xC8, 0xBC, 0x8A, 0x41, 0x78, 0x70, 0xD9, 0x6D, 0x97, 0xB3, 0x4A, 0x8B,
    0x78, 0x2D, 0x5E, 0xB4, 0x0F, 0xA3, 0x4C, 0x60, 0xCA, 0xE1, 0x47, 0xCB,
    0x78, 0x2D, 0x12, 0x17, 0xB1, 0x52, 0x8B, 0xCA, 0x39, 0x2C, 0xBD, 0xB5,
    0x2F, 0xC2, 0x33, 0x02, 0x96, 0xAB, 0xDA, 0x94, 0x7F,
};

bool NET_PRES_CertStoreGetCACerts(const uint8_t ** certPtr, int32_t * certSize, uint8_t certIndex)
{
    switch (certIndex) {
        case 0:
            *certPtr = ISRG_Root_X1;
            *certSize = sizeof(ISRG_Root_X1);
            break;

        case 1:
            *certPtr = DigiCert_Global_Root_G2;
            *certSize = sizeof(DigiCert_Global_Root_G2);
            break;

        case 2:
            *certPtr = AMAZON_ROOT_CA_1;
            *certSize = sizeof(AMAZON_ROOT_CA_1);
            break;

        case 3:
            *certPtr = Baltimore_CyberTrust_Root;
            *certSize = sizeof(Baltimore_CyberTrust_Root);
            break;

        default:
            return false;
    }
    return true;
}
