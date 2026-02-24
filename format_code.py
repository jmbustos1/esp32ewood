import re
import sys

def format_c_code(input_code):
    # Eliminar prefijo
    if input_code.startswith('este es el codigo: '):
        input_code = input_code[len('este es el codigo: '):]
    
    # Proteger strings con escapes
    strings = []
    def save_string(m):
        idx = len(strings)
        strings.append(m.group(0))
        return f'__STR{idx}__'
    
    # Proteger strings (manejar escapes)
    code = re.sub(r'"(?:[^"\\]|\\.)*"', save_string, input_code)
    
    # Proteger comentarios
    comments = []
    def save_comment(m):
        idx = len(comments)
        comments.append(m.group(0))
        return f'__COM{idx}__'
    
    code = re.sub(r'//[^\n]*', save_comment, code)
    
    # Ahora formatear sin strings/comentarios
    
    # Separar includes
    code = re.sub(r'(#include\s+[<"][^>"]+[>"])', r'\1\n', code)
    
    # Separar defines  
    code = re.sub(r'(#define\s+\w+[^\n]*)', r'\1\n', code)
    
    # Separar typedef
    code = re.sub(r'(typedef\s+[^;]+;)', r'\1\n', code)
    
    # Separar static
    code = re.sub(r'(static\s+[^;]+;)', r'\1\n', code)
    
    # Separar por punto y coma
    parts = code.split(';')
    formatted = []
    for i, p in enumerate(parts):
        p = p.strip()
        if p:
            formatted.append(p + (';' if i < len(parts)-1 else ''))
    code = '\n'.join(formatted)
    
    # Separar llaves
    code = re.sub(r'\{', r'\n{\n', code)
    code = re.sub(r'\}', r'\n}\n', code)
    
    # Restaurar comentarios
    for i, com in enumerate(comments):
        code = code.replace(f'__COM{i}__', com)
    
    # Restaurar strings
    for i, s in enumerate(strings):
        code = code.replace(f'__STR{i}__', s)
    
    # Limpiar espacios múltiples
    code = re.sub(r' +', ' ', code)
    code = re.sub(r'\n{3,}', '\n\n', code)
    
    return code

if __name__ == '__main__':
    if len(sys.argv) > 1:
        with open(sys.argv[1], 'r') as f:
            code = f.read()
    else:
        code = sys.stdin.read()
    
    formatted = format_c_code(code)
    print(formatted, end='')
