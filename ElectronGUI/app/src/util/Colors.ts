export type ColorTypes =
{Primary: '#FF7A00'} | {Secondary: '#FF9839'} | {Tertiary: '#0E2C85'} | {Quaternary: '#1339AC'}  | {Quinary: '#03899C'} | {Success: '#2DC748'} | {Error: '#FF1300'} | {Warning: '#FFAE00'} | {Text: '#111111'} | {Underline: '#d4d4d4'};
export const Colors =  {
  Primary: '#FF7A00' as '#FF7A00' ,
  Secondary: '#FF9839',
  Tertiary: '#0E2C85',
  Quaternary: '#1339AC',
  Quinary: '#03899C',
  Success: '#2DC748',
  Error: '#FF1300',
  Warning: '#FFAE00',
  Text: '#111111',
  Underline: '#d4d4d4'
};


// TODO: Figure out a better way to do background colors
export const backgroundColors = ['rgba(255,255,255, 1)', 'rgba(255,0,0,1)', 'rgba(0,0,255,1)', 'rgba(0,0,0,1)', 'rgba(0,255,0,1)', 'rgba(255,255,0,1)', 'rgba(255,153,0,1)'];

export const opaqueBackgroundColors = ['rgba(255,255,255, 0.2)', 'rgba(255,0,0,0.2)', 'rgba(0,0,255,0.2)', 'rgba(0,0,0,0.2)', 'rgba(0,255,0,0.2)', 'rgba(255,255,0,0.2)', 'rgba(255,153,0,0.2)'];