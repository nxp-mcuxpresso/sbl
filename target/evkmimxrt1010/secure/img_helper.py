import click
import os

class BasedIntParamType(click.ParamType):
    name = 'integer'

    def convert(self, value, param, ctx):
        try:
            if value[:2].lower() == '0x':
                return int(value[2:], 16)
            elif value[:1] == '0':
                return int(value, 8)
            return int(value, 10)
        except ValueError:
            self.fail('%s is not a valid integer' % value, param, ctx)

@click.group()
def cli():
    pass

@click.command()
@click.option('--image', required=True, help='image.')
@click.option('--size', required=True, help='Deleted size.')
def deleteheader(image, size):
    """Delete the given bytes from application header."""

    folder = os.path.dirname(image)

    with open(image, "rb") as f_r:
        temp_bin = os.path.join(folder, 'temp')
        with open(temp_bin, "wb") as f_w:
            c = f_r.read()
            f_w.write(c[int(size):])

    os.remove(image)
    os.rename(temp_bin, image)

@click.command()
@click.option('--pad-size', required=True, type=BasedIntParamType(),
              help='Size of the padding')
@click.option('--input', required=True, help='Encrypted image.')
@click.option('--output', required=True, help='output image.')
def paddingimage(pad_size, input, output):
    """Padding image in header for image."""
    
    with open(output, "wb") as f_w:
        f_w.write(b'\x00' * pad_size)

        with open(input, "rb") as f_r:
            c = f_r.read()

        f_w.write(c)
        

@click.command()
@click.option('--header-size', type=BasedIntParamType(), help='MCUBOOT header size.')
@click.option('--sign-image', required=True, help='Encrypted image.')
@click.option('--enc-image', required=True, help='Signed image.')
def merge(sign_image, enc_image, header_size = 0x1000):
    """Merge signed image and encrypted to generate final application."""

    with open("temp123.bin", "wb") as f_w:
        with open(sign_image, "rb") as f_r:
            c = f_r.read(header_size)
            f_w.write(c)
            
        with open(enc_image, "rb") as f_r:
            # skip firt 4k bytes, copy real image data from 0x1000
            f_r.seek(header_size)
            body = f_r.read()

            f_w.write(body)

        with open(sign_image, "rb") as f_r:
            f_r.seek(header_size + len(body))
            tlv_content = f_r.read()

            f_w.write(tlv_content)

    os.remove(enc_image)
    os.rename("temp123.bin", enc_image)
    
@click.command()
@click.option('--type', required=True, help='Encrypted image.')
@click.option('--enc_image', required=True, help='Encrypted image.')
@click.option('--output', required=True, help='output image.')
def extract_keycontext(type, enc_image, output):
    """Extract key context from encrypted image."""
    
    key_info_size = 0x100
    key_info_offset = 0
    
    if type == 'bee':
        key_info_size = 0x180
        key_info_offset = 0x800
        
    with open(enc_image, "rb") as f_r:
        f_r.seek(key_info_offset)
        c = f_r.read(key_info_size)
        
    with open(output, "wb") as f_w:
        f_w.write(c)

cli.add_command(deleteheader)
cli.add_command(paddingimage)
cli.add_command(merge)
cli.add_command(extract_keycontext)

if __name__ == '__main__':
    cli()