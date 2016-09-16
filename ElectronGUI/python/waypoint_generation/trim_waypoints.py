import math
from geoLocation import Location


class TrimWaypoints:

    # testSmall = [[[37.88997973925902, -76.81525131879368], [37.8897821801886, -76.81513477514744], [37.8896000894016, -76.81502819492404], [37.8896000894016, -79.81502819492404], [37.8896000894016, -79.81502819492404], [37.8896000894016, -79.81502819492404], [37.8896000894016, -76.81502819492404]]]
    # argboundaryVertices = [[37.88997973925902, -76.81525131879368], [37.8897821801886, -76.81513477514744], [37.8896000894016, -76.81502819492404], [37.88943229012433, -76.81493279679238], [37.88927736584623, -76.81484984516747], [37.8891336384791, -76.81478059827865], [37.88899916007118, -76.81472623878412], [37.8888717247845, -76.81468779154905], [37.88874890615146, -76.81466603848177], [37.88862812049977, -76.81466144458713], [37.888506711551884, -76.8146741102236], [37.88838204550843, -76.81470376041831], [37.88825160283777, -76.81474977378798], [37.888113053982366, -76.81481124426753], [37.88796431077531, -76.8148870622266], [37.88780355144765, -76.81497599980929], [37.887629222338695, -76.81507678805292], [37.88744002241338, -76.81518817837429], [37.88744002241338, -76.81518817837429], [37.887428427832994, -76.81516147032131], [37.88741677447012, -76.81513505386432], [37.887405058696785, -76.81510892459382], [37.8873932768889, -76.81508307802157], [37.887381425427925, -76.81505750958168], [37.88736950070247, -76.81503221463151], [37.887357499109996, -76.81500718845282], [37.88734541705845, -76.81498242625295], [37.887333250967934, -76.81495792316618], [37.88732099727238, -76.81493367425507], [37.887308652421176, -76.81490967451205], [37.8872962128808, -76.81488591886094], [37.887283675136416, -76.81486240215862], [37.88727103569357, -76.8148391191969], [37.88725829107967, -76.81481606470426], [37.88724543784563, -76.81479323334791], [37.88723247256734, -76.81477061973573], [37.88723247256734, -76.81477061973573], [37.88747541534583, -76.81463179364992], [37.88769417299553, -76.81450364251488], [37.88789025858911, -76.81438815062529], [37.888065668685336, -76.814287547106], [37.888222989757075, -76.81420421662281], [37.88836547967483, -76.81414053085331], [37.888497085419395, -76.81409859962255], [37.888622356875494, -76.81407997723497], [37.88874623860591, -76.81408540082582], [37.888873766447006, -76.81411465252877], [37.8890097403993, -76.81416660116994], [37.889158457466486, -76.81423940614418], [37.88932355808137, -76.81433080497446], [37.88950799087074, -76.81443839406185], [37.88971406438858, -76.81455984323922], [37.88994354399074, -76.81469302695382], [37.89019776009257, -76.81483608382943], [37.89019776009257, -76.81483608382943], [37.89021155126296, -76.81481367663264], [37.890225470721006, -76.81479146016812], [37.89023952163068, -76.81476942888962], [37.89025370712309, -76.81474757720952], [37.89026803029532, -76.81472589950128], [37.890282494209366, -76.8147043901018], [37.89029710189103, -76.81468304331376], [37.89031185632887, -76.8146618534082], [37.890326760473336, -76.81464081462691], [37.890341817235736, -76.81461992118501], [37.89035702948741, -76.81459916727339], [37.890372400058915, -76.81457854706133], [37.89038793173922, -76.81455805469905], [37.89040362727502, -76.81453768432024], [37.89041948937005, -76.81451743004459], [37.890435520684456, -76.81449728598044], [37.89045172383427, -76.8144772462273], [37.89045172383427, -76.8144772462273], [37.89012746600358, -76.8143172369849], [37.88983902564083, -76.81416536010656], [37.889585253749665, -76.8140237893916], [37.889364440913056, -76.81389530295148], [37.889174051076566, -76.8137833476963], [37.88901038437926, -76.81369197213805], [37.888868236524054, -76.81362549997596], [37.888740721965064, -76.81358784994926], [37.88861949482422, -76.81358160240988], [37.88849548696475, -76.81360721443546], [37.88835995726879, -76.81366286650542], [37.888205395593076, -76.81374504720387], [37.88802594603993, -76.81384949032838], [37.88781735819064, -76.81397198236363], [37.887576684672425, -76.8141088221564], [37.887301929695056, -76.81425697320607], [37.886991752055835, -76.81441403736767], [37.886991752055835, -76.81441403736767], [37.8869981648942, -76.81438370140539], [37.88700453345847, -76.8143535921806], [37.88701085723478, -76.81432370853668], [37.88701713570495, -76.81429404929457], [37.887023368346654, -76.8142646132525], [37.88702955463332, -76.81423539918548], [37.88703569403434, -76.81420640584497], [37.887041786015, -76.81417763195863], [37.8870478300367, -76.81414907622984], [37.88705382555702, -76.81412073733739], [37.88705977202968, -76.81409261393512], [37.88706566890479, -76.81406470465153], [37.887071515628875, -76.81403700808946], [37.88707731164501, -76.81400952282573], [37.8870830563929, -76.81398224741075], [37.887088749309065, -76.81395518036824], [37.887094389826885, -76.81392832019478], [37.887094389826885, -76.81392832019478], [37.8874000703311, -76.81379051997843], [37.8876690258018, -76.81365886296405], [37.887902172616165, -76.81353534520952], [37.88810098814941, -76.81342270187118], [37.88826787102907, -76.81332459243222], [37.88840666985917, -76.81324564253885], [37.88852330490086, -76.81319106855055], [37.88862615176092, -76.8131655948247], [37.888725622238546, -76.81317183115956], [37.88883269899679, -76.81320911112809], [37.88895712069085, -76.81327380660893], [37.8891062979454, -76.8133608245203], [37.88928527294355, -76.8134650933167], [37.88949725584864, -76.81358235030208], [37.889744232327516, -76.81370931425373], [37.89002742666704, -76.81384356282005], [37.89034760273092, -76.81398333489251], [37.89034760273092, -76.81398333489251], [37.890341946359776, -76.81395615022983], [37.89033634548586, -76.81392916385389], [37.89033080066622, -76.81390237415603], [37.89032531245964, -76.81387577950069], [37.89031988142655, -76.81384937822514], [37.89031450812885, -76.81382316863925], [37.89030919312975, -76.8137971490252], [37.89030393699352, -76.8137713176373], [37.89029874028541, -76.81374567270173], [37.89029360357137, -76.81372021241627], [37.89028852741793, -76.81369493495019], [37.89028351239196, -76.81366983844397], [37.89027855906042, -76.81364492100913], [37.89027366799026, -76.81362018072804], [37.89026883974809, -76.8135956156538], [37.890264074900024, -76.81357122380999], [37.89025937401141, -76.81354700319062], [37.89025937401141, -76.81354700319062], [37.88994173367869, -76.81343160020478], [37.889661919043405, -76.81331936826936], [37.88941952509143, -76.81321150372082], [37.88921384552138, -76.81310979468638], [37.88904360325336, -76.81301693587697], [37.88890643154558, -76.81293692757708], [37.88879796108421, -76.81287533794361], [37.88871058602663, -76.81283875156522], [37.888632803547026, -76.8128325367372], [37.88855073364912, -76.81285774553714], [37.88845161292739, -76.81291037667147], [37.88832630698, -76.81298395506], [37.88816933171881, -76.81307237193806], [37.88797767244745, -76.81317102284825], [37.887749707550576, -76.81327675214435], [37.88748455059262, -76.81338746142963], [37.88718170070779, -76.81350175353631], [37.88718170070779, -76.81350175353631], [37.88718629856834, -76.8134783271167], [37.88719083289409, -76.81345507343651], [37.88719530309357, -76.81343199038766], [37.88719970857641, -76.81340907582936], [37.88720404875369, -76.81338632758812], [37.88720832303822, -76.81336374345759], [37.88721253084479, -76.81334132119841], [37.88721667159054, -76.81331905853818], [37.887220744695185, -76.81329695317146], [37.88722474958139, -76.81327500275962], [37.88722868567505, -76.81325320493099], [37.88723255240563, -76.81323155728066], [37.88723634920647, -76.81321005737071], [37.88724007551514, -76.81318870273017], [37.887243730773804, -76.81316749085515], [37.887247314429466, -76.81314641920888], [37.8872508259344, -76.81312548522187], [37.8872508259344, -76.81312548522187], [37.88755223079252, -76.81303921392454], [37.887815367805665, -76.81295451726889], [37.88804042302015, -76.8128721183776], [37.88822776034404, -76.81279321516115], [37.88837814630492, -76.81271987438606], [37.88849331649139, -76.81265574478887], [37.88857736685785, -76.8126070353626], [37.888639444408895, -76.81258242738296], [37.88869568649178, -76.81258859107733], [37.88876479404937, -76.81262366594213], [37.88886024622387, -76.81267931447557], [37.88898884097688, -76.81274760378558], [37.88915350639725, -76.81282342969452], [37.88935549467752, -76.81290387005741], [37.88959537383475, -76.81298726143159], [37.88987342158368, -76.81307262525767], [37.890189783794014, -76.81315936091399], [37.890189783794014, -76.81315936091399], [37.89018632847385, -76.81313784509253], [37.89018294736717, -76.81311645667836], [37.89017964096809, -76.81309519306798], [37.89017640976408, -76.81307405162926], [37.89017325423562, -76.81305302970166], [37.89017017485591, -76.81303212459662], [37.89016717209053, -76.81301133359771], [37.89016424639711, -76.81299065396118], [37.89016139822506, -76.81297008291622], [37.89015862801525, -76.81294961766535], [37.89015593619968, -76.81292925538486], [37.890153323201176, -76.81290899322532], [37.89015078943309, -76.81288882831194], [37.890148335299024, -76.81286875774516], [37.89014596119247, -76.81284877860112], [37.8901436674966, -76.8128288879322], [37.890141454583876, -76.81280908276763], [37.890141454583876, -76.81280908276763], [37.88982554085361, -76.81275493043591], [37.88954813340966, -76.81270111645068], [37.88930920786066, -76.81264780713481], [37.889108714037214, -76.81259529481618], [37.888946536341145, -76.81254413751225], [37.88882236570779, -76.81249551539878], [37.88873520380131, -76.81245222591247], [37.88868116931049, -76.8124212526979], [37.888646051652806, -76.81241526272318], [37.8886020198275, -76.81243807558836], [37.888528566799835, -76.81247775758017], [37.88841902503529, -76.81252479162939], [37.88827172786157, -76.81257516711258], [37.88808619872151, -76.81262725307023], [37.887862271295354, -76.81268031188058], [37.88759987714128, -76.81273397129348], [37.88729898431088, -76.81278802568703], [37.88729898431088, -76.81278802568703], [37.887301111539756, -76.81276909439619], [37.88730315767928, -76.81275024610437], [37.887305122360935, -76.8127314776581], [37.88730700522883, -76.8127127858805], [37.887308805939895, -76.81269416757192], [37.887310524164256, -76.81267561951084], [37.88731215958545, -76.81265713845463], [37.8873137119008, -76.81263872114037], [37.88731518082161, -76.81262036428576], [37.887316566073444, -76.81260206459], [37.88731786739641, -76.81258381873458], [37.88731908454536, -76.81256562338443], [37.88732021729009, -76.81254747518865], [37.887321265415665, -76.81252937078165], [37.88732222872247, -76.81251130678403], [37.88732310702652, -76.81249327980365], [37.8873239001596, -76.81247528643665], [37.8873239001596, -76.81247528643665], [37.8876247096302, -76.8124565098056], [37.88788697924452, -76.81243775120201], [37.8881107094968, -76.81241901998892], [37.88829590150767, -76.81240033405949], [37.88844255828206, -76.81238173237018], [37.88855069026449, -76.81236331827803], [37.88862035511967, -76.81234547651968], [37.88865244136413, -76.81233097708922], [37.8886684840564, -76.81233565660455], [37.88871488586824, -76.81235230318396], [37.88879953694943, -76.81237045281824], [37.888922691269855, -76.81238895107815], [37.88908437621962, -76.8124075799692], [37.889284597969336, -76.81242627099178], [37.889523358513635, -76.81244499598051], [37.889800658652725, -76.81246374127466], [37.89011649875777, -76.81248249946324], [37.89011649875777, -76.81248249946324], [37.890115789920706, -76.81246370295801], [37.89011516658251, -76.81244493202894], [37.890114628852665, -76.81242618339449], [37.890114176825726, -76.81240745376661], [37.890113810581305, -76.81238873985171], [37.89011353018393, -76.81237003835152], [37.890113335683004, -76.81235134596416], [37.89011322711275, -76.81233265938505], [37.89011320449222, -76.81231397530789], [37.89011326782527, -76.81229529042565], [37.890113417100494, -76.81227660143155], [37.89011365229132, -76.81225790501999], [37.890113973355994, -76.81223919788759], [37.89011438023759, -76.81222047673411], [37.89011487286412, -76.81220173826341], [37.89011545114854, -76.81218297918448], [37.89011611498888, -76.81216419621235], [37.89011611498888, -76.81216419621235], [37.88980027603112, -76.8121818810113], [37.889522977144395, -76.81219955213001], [37.88928421803658, -76.81221720338112], [37.88908399807751, -76.81223482334829], [37.88892231569462, -76.81225238846052], [37.88879916601128, -76.81226984124717], [37.888714527243415, -76.81228699771319], [37.888668191694116, -76.81230286055516], [37.88865261892623, -76.81230741733526], [37.88862069898825, -76.81229347977245], [37.888551053586504, -76.8122765958998], [37.88844292618652, -76.81225923121318], [37.888296270696735, -76.81224171210411], [37.88811107885745, -76.81222412543976], [37.887887348301064, -76.81220650385409], [37.88762507814627, -76.81218886231389], [37.887324268004775, -76.8121712086454], [37.887324268004775, -76.8121712086454], [37.88732351789964, -76.81215323132677], [37.8873226825414, -76.81213522215987], [37.887321762089954, -76.81211717773512], [37.88732075672102, -76.8120990946542], [37.88731966662599, -76.81208096953102], [37.88731849201179, -76.81206279899274], [37.887317233100575, -76.81204457968074], [37.887315890129656, -76.81202630825165], [37.887314463351196, -76.81200798137829], [37.887312953032016, -76.81198959575062], [37.887311359453285, -76.81197114807665], [37.88730968291039, -76.81195263508334], [37.88730792371253, -76.81193405351749], [37.88730608218253, -76.81191540014655], [37.88730415865658, -76.8118966717595], [37.887302153483795, -76.81187786516759], [37.887300067026096, -76.81185897720516], [37.887300067026096, -76.81185897720516], [37.8876009561047, -76.81191197475076], [37.88786334338163, -76.81196460244169], [37.88808725850238, -76.81201666552633], [37.888272765128896, -76.81206781062149], [37.88842001858371, -76.81211733482274], [37.88852946713133, -76.81216367333198], [37.888602700754156, -76.81220292797529], [37.888646245917414, -76.81222563667907], [37.888680737461755, -76.81221965457789], [37.88873440040633, -76.8121888913793], [37.88882140208396, -76.81214613494424], [37.88894549930249, -76.8120982595882], [37.889107638398606, -76.81204796765623], [37.88930810922576, -76.81199638782297], [37.88954701959623, -76.81194405089961], [37.889824416153466, -76.81189123409968], [37.89014032153096, -76.81183809484106], [37.89014032153096, -76.81183809484106], [37.890142491670325, -76.81181833238777], [37.89014474277624, -76.81179848596304], [37.89014707448168, -76.8117785525844], [37.89014948640877, -76.81175852929229], [37.89015197816917, -76.8117384131506], [37.890154549364304, -76.81171820124713], [37.89015719958565, -76.81169789069438], [37.89015992841509, -76.81167747862978], [37.89016273542516, -76.81165696221633], [37.890165620179374, -76.81163633864307], [37.890168582232555, -76.81161560512541], [37.89017162113112, -76.81159475890566], [37.8901747364134, -76.81157379725325], [37.89017792760996, -76.81155271746532], [37.890181194243894, -76.81153151686686], [37.890184535831146, -76.81151019281113], [37.890187951880854, -76.81148874267986], [37.890187951880854, -76.81148874267986], [37.889871617441976, -76.81157457053423], [37.88959360701765, -76.81165906655743], [37.88935378041923, -76.81174164665916], [37.88915187006496, -76.8118213567986], [37.88898732658854, -76.81189656917849], [37.88885893196938, -76.81196440836914], [37.88876381361975, -76.81201981066185], [37.88869522524206, -76.8120548206049], [37.888639632232646, -76.81206098631135], [37.88857813860537, -76.81203641219255], [37.88849448605876, -76.81198788333111], [37.88837955399312, -76.81192414622741], [37.888229307826066, -76.81185138576411], [37.888042054711704, -76.81177320281691], [37.887817052048675, -76.81169162383722], [37.88755394887041, -76.81160781827501], [37.88725256628961, -76.8115224900025], [37.88725256628961, -76.8115224900025], [37.8872490911082, -76.8115016262153], [37.88724554348907, -76.81148062611139], [37.88724192397697, -76.81145948710449], [37.88723823312208, -76.81143820664087], [37.88723447147964, -76.81141678219967], [37.887230639609676, -76.81139521129289], [37.88722673807654, -76.8113734914655], [37.887222767448726, -76.81135162029557], [37.887218728298386, -76.81132959539424], [37.88721462120113, -76.81130741440582], [37.88721044673563, -76.81128507500767], [37.88720620548334, -76.81126257491039], [37.8872018980282, -76.81123991185754], [37.88719752495629, -76.8112170836257], [37.88719308685557, -76.81119408802442], [37.887188584315574, -76.81117092289604], [37.887184017927154, -76.81114758611561], [37.887184017927154, -76.81114758611561], [37.887486811359786, -76.81126107062836], [37.88775188842142, -76.81137104616077], [37.88797973842206, -76.81147613579951], [37.888171229953976, -76.8115742654876], [37.888327958392665, -76.81166230394678], [37.88845290523023, -76.81173565901678], [37.888551528618585, -76.8117882009353], [37.88863297926696, -76.8118133915523], [37.88871010577339, -76.81180717135585], [37.888796899058235, -76.81177060761135], [37.888904921181364, -76.8117091363342], [37.889041773477274, -76.81162938929856], [37.889211794019126, -76.81153693931527], [37.88941731876018, -76.81143576714736], [37.88965960224653, -76.81132854206031], [37.889939335856944, -76.81121702912094], [37.890256915003675, -76.8111024061565], [37.890256915003675, -76.8111024061565], [37.89026158148586, -76.81107827258555], [37.89026631220975, -76.81105396883011], [37.89027110661036, -76.8110294928689], [37.89027596412186, -76.81100484271032], [37.89028088417774, -76.81098001639235], [37.89028586621113, -76.81095501198233], [37.89029090965493, -76.81092982757683], [37.890296013942084, -76.81090446130149], [37.890301178505794, -76.8108789113108], [37.8903064027797, -76.81085317578793], [37.89031168619804, -76.8108272529445], [37.890317028195966, -76.8108011410204], [37.89032242820957, -76.81077483828354], [37.890327885676214, -76.81074834302957], [37.890333400034585, -76.81072165358174], [37.89033897072493, -76.81069476829057], [37.890344597189234, -76.81066768553362], [37.890344597189234, -76.81066768553362], [37.89002452675947, -76.81080680777232], [37.889741468421086, -76.81094048000682], [37.88949466985514, -76.81106695642312], [37.889282922364686, -76.81118382996026], [37.88910425969054, -76.81128783047095], [37.88895549161679, -76.81137469466863], [37.8888315871076, -76.81143933038986], [37.88872512241854, -76.81147660413276], [37.88862631083398, -76.81148284810604], [37.8885240992582, -76.81145738838406], [37.888408016271114, -76.81140286682037], [37.888269660346516, -76.81132405291103], [37.88810311541522, -76.81122619329432], [37.88790455138595, -76.81111392182163], [37.88767159027963, -76.81099089078944], [37.88740277221218, -76.81085982126847], [37.887097194065745, -76.8107226938523], [37.887097194065745, -76.8107226938523], [37.88709157940381, -76.81069594013208], [37.8870859120399, -76.8106689803324], [37.88708019254209, -76.81064181293574], [37.88707442147546, -76.81061443645186], [37.88706859940184, -76.8105868494174], [37.88706272687987, -76.81055905039558], [37.887056804464706, -76.81053103797582], [37.88705083270807, -76.81050281077339], [37.88704481215801, -76.8104743674291], [37.887038743358886, -76.81044570660887], [37.88703262685124, -76.81041682700341], [37.88702646317169, -76.81038772732785], [37.88702025285294, -76.81035840632143], [37.88701399642355, -76.81032886274701], [37.88700769440802, -76.81029909539086], [37.887001347326596, -76.8102691030622], [37.8869949556953, -76.81023888459288], [37.8869949556953, -76.81023888459288], [37.88730497919158, -76.81039539680067], [37.88757953620643, -76.81054308299399], [37.88781995553493, -76.81067955393736], [37.88802821892259, -76.81080177775368], [37.88820726047438, -76.81090604963536], [37.888361323191916, -76.81098814040993], [37.88849626884715, -76.81104375639116], [37.888619632928354, -76.8110693551741], [37.88874019967953, -76.81106309802061], [37.88886708560365, -76.81102544608599], [37.8890086739721, -76.81095900406099], [37.889171868101634, -76.81086772075719], [37.889361872487946, -76.81075594236984], [37.88958237691618, -76.81062772748581], [37.88983590378055, -76.81048652251722], [37.89012414920543, -76.81033509858237], [37.89044825089193, -76.81017561955814], [37.89044825089193, -76.81017561955814], [37.89043203215251, -76.81015565719348], [37.89041598601462, -76.81013559226427], [37.89040010986913, -76.81011541885448], [37.89038440106245, -76.81009513105617], [37.89036885689704, -76.81007472297216], [37.890353474631986, -76.81005418871852], [37.89033825148374, -76.81003352242719], [37.89032318462674, -76.8100127182485], [37.89030827119422, -76.80999177035385], [37.890293508279036, -76.80997067293816], [37.89027889293451, -76.80994942022252], [37.890264422175335, -76.80992800645667], [37.89025009297859, -76.80990642592147], [37.890235902284694, -76.80988467293147], [37.8902218469985, -76.8098627418373], [37.890207923990395, -76.80984062702812], [37.89019413009745, -76.80981832293392], [37.89019413009745, -76.80981832293392], [37.88994013538815, -76.80996107892337], [37.88971091763623, -76.81009402154795], [37.88950515291981, -76.81021529020192], [37.88932108163659, -76.810322756778], [37.889156398795166, -76.81041408433722], [37.88900815521469, -76.81048685818168], [37.888872703850915, -76.81053880177708], [37.888745733986084, -76.81056806066134], [37.888622425669155, -76.81057349422794], [37.888497719991754, -76.8105548830361], [37.88836665061131, -76.81051297122391], [37.88822465103474, -76.81044932623331], [37.88806776472496, -76.81036607298775], [37.88789273098627, -76.81026559640469], [37.88769696555503, -76.81015029034982]]

    # print "The original length is: %s" %len(argboundaryVertices[0])

    def __init__(self, bearingThreshold):
        self.bearingThreshold = bearingThreshold

    def trimWPArray(self, wpArray):
        print "The original array length: %s" %len(wpArray)
        geoLocationArray = self.createGeoArray(wpArray)
        reducedArray = self.computeReducedWPSet(geoLocationArray)
        trimmedArray = self.createWPArray(reducedArray)
        print "The trimmed array length is: %s" %len(trimmedArray)

        return trimmedArray

    def createGeoArray(self, locationArray):
        geoLocationArray = []
        for i in range(0,len(locationArray),1):
            acutalPosition = locationArray[i]
            lat = acutalPosition[0]
            lng = acutalPosition[1]
            locationObject = Location(lat,lng)
            geoLocationArray.append(locationObject)
        return geoLocationArray

    def createWPArray(self, locationArray):
        wpArray = []
        for point in locationArray:
            wpArray.append([point.latitude, point.longitude])

        return wpArray

    def computeReducedWPSet(self, geoLocationArray):
        reducedArray = []
        if(len(geoLocationArray) == 0):
            return reducedArray

        reducedArray.append(geoLocationArray[0])
        reducedArray.append(geoLocationArray[1])
        for i in xrange(2, len(geoLocationArray)-1):
            oldBearing = reducedArray[len(reducedArray)-2].bearingBetween(reducedArray[len(reducedArray)-1])
            nextBearing = reducedArray[len(reducedArray)-1].bearingBetween(geoLocationArray[i])
            if oldBearing < 0:
                oldBearing = oldBearing + 360
            if nextBearing < 0:
                nextBearing = nextBearing + 360
            tmpBearingDelta = math.fabs(nextBearing - oldBearing)

            if tmpBearingDelta > self.bearingThreshold:
                reducedArray.append(geoLocationArray[i-1])
                # reducedArray.append(geoLocationArray[i])

        return reducedArray
